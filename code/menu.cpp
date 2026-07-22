#include "menu.hpp"
#include "Key.hpp"
#include "init.hpp"
#include "motor.hpp"
#include "MyFlash.hpp"
#include "avoid.hpp"
#include "beep.hpp"
#include "ips200_draw.hpp"
#include "StartLine.hpp"
#include "Test.hpp"

extern void image_transfer_set_enabled(bool enable);
extern bool image_transfer_is_enabled(void);

#define MENU_TOP_COUNT 4
#define MOTOR_ITEM_COUNT MOTOR_SPEED_PARAM_COUNT
#define PID_ITEM_COUNT MOTOR_TURN_PARAM_COUNT
#define TEST_ITEM_COUNT 7
#define BASE_SPEED_STEP 5
#define TOWPOINT_STEP 1
#define STARTLINE_TARGET_STEP 1

// 一级菜单：Speed / PID / Avoid / Test
// Speed 用于调基础速度和各档弯道限速倍率。
typedef enum
{
    MENU_PAGE_TOP = 0,
    MENU_PAGE_MOTOR,
    MENU_PAGE_PID,
    MENU_PAGE_AVOID,
    MENU_PAGE_TEST
} MenuPage;

static MenuPage menu_page = MENU_PAGE_TOP;
static int top_index = 0;
static int motor_index = 0;
static bool base_speed_selected = false;
static int pid_index = 0;
static bool pid_param_selected = false;
static int avoid_index = 0;
static bool avoid_param_selected = false;
static int test_index = 0;
static bool towpoint_selected = false;
static bool menu_dirty = true;

static void menu_mark_dirty(void)
{
    menu_dirty = true;
}

static void menu_wrap_index(int *value, int count)
{
    if (*value < 0)
        *value = count - 1;
    if (*value >= count)
        *value = 0;
}

static void draw_red_marker(uint16 x, uint16 y)
{
    for (uint16 dy = 0; dy < 12; dy++)
    {
        for (uint16 dx = 0; dx < 8; dx++)
        {
            ips200.draw_point(x + dx, y + dy, RGB565_RED);
        }
    }
}

void Menu_StopToMenu(void)
{
    stop_car();
    menu_page = MENU_PAGE_TOP;
    base_speed_selected = false;
    avoid_param_selected = false;
    pid_param_selected = false;
    towpoint_selected = false;
    menu_mark_dirty();
}

void Menu_Process(void)
{
    Key_Process();
    int key = Key_GetValueOnce();

    if (key == KEY_NONE)
        return;

    menu_mark_dirty();

    if (car_start_flag)
    {
        if (key == KEY_4)
        {
            Menu_StopToMenu();
            printf("[MENU] KEY4 stop, return menu\r\n");
        }
        return;
    }

    if (menu_page == MENU_PAGE_TOP)
    {
        if (key == KEY_1)
        {
            top_index--;
            menu_wrap_index(&top_index, MENU_TOP_COUNT);
        }
        else if (key == KEY_2)
        {
            top_index++;
            menu_wrap_index(&top_index, MENU_TOP_COUNT);
        }
          else if (key == KEY_3)
          {
              if (top_index == 0)
                  menu_page = MENU_PAGE_MOTOR;
              else if (top_index == 1)
                  menu_page = MENU_PAGE_PID;
              else if (top_index == 2)
                  menu_page = MENU_PAGE_AVOID;
              else
                  menu_page = MENU_PAGE_TEST;
              motor_index = 0;
              pid_index = 0;
              avoid_index = 0;
              test_index = 0;
              base_speed_selected = false;
              pid_param_selected = false;
              avoid_param_selected = false;
              towpoint_selected = false;
          }
        else if (key == KEY_4)
        {
            printf("[MENU] KEY4 launch\r\n");
            start_car();
            menu_mark_dirty();
        }
    }
    else if (menu_page == MENU_PAGE_MOTOR)
    {
        if (base_speed_selected)
        {
            if (key == KEY_1)
            {
                motor_adjust_speed_param(motor_index, 1);
            }
            else if (key == KEY_2)
            {
                motor_adjust_speed_param(motor_index, -1);
            }
            else if (key == KEY_4)
            {
                MyFlash_SaveParameters();
                base_speed_selected = false;
                printf("[MENU] speed %s=%.1f saved\r\n",
                       motor_get_speed_param_name(motor_index), motor_get_speed_param_value(motor_index));
            }
        }
        else
        {
            if (key == KEY_1)
            {
                motor_index--;
                menu_wrap_index(&motor_index, MOTOR_ITEM_COUNT);
            }
            else if (key == KEY_2)
            {
                motor_index++;
                menu_wrap_index(&motor_index, MOTOR_ITEM_COUNT);
            }
            else if (key == KEY_3)
            {
                base_speed_selected = true;
            }
            else if (key == KEY_4)
            {
                menu_page = MENU_PAGE_TOP;
                base_speed_selected = false;
            }
        }
    }
    else if (menu_page == MENU_PAGE_PID)
    {
        if (pid_param_selected)
        {
            if (key == KEY_1)
            {
                motor_adjust_turn_param(pid_index, 1);
            }
            else if (key == KEY_2)
            {
                motor_adjust_turn_param(pid_index, -1);
            }
            else if (key == KEY_4)
            {
                MyFlash_SaveParameters();
                pid_param_selected = false;
                printf("[MENU] pid %s=%.1f saved\r\n",
                       motor_get_turn_param_name(pid_index), motor_get_turn_param_value(pid_index));
            }
        }
        else
        {
            if (key == KEY_1)
            {
                pid_index--;
                menu_wrap_index(&pid_index, PID_ITEM_COUNT);
            }
            else if (key == KEY_2)
            {
                pid_index++;
                menu_wrap_index(&pid_index, PID_ITEM_COUNT);
            }
            else if (key == KEY_3)
            {
                pid_param_selected = true;
            }
            else if (key == KEY_4)
            {
                menu_page = MENU_PAGE_TOP;
                pid_param_selected = false;
            }
        }
    }
    else if (menu_page == MENU_PAGE_AVOID)
    {
        if (avoid_param_selected)
        {
            if (key == KEY_1)
            {
                avoid_adjust_param(avoid_index, 1);
            }
            else if (key == KEY_2)
            {
                avoid_adjust_param(avoid_index, -1);
            }
            else if (key == KEY_4)
            {
                MyFlash_SaveParameters();
                avoid_param_selected = false;
                printf("[MENU] avoid %s=%.1f saved\r\n", avoid_get_param_name(avoid_index), avoid_get_param_value(avoid_index));
            }
        }
        else
        {
            if (key == KEY_1)
            {
                avoid_index--;
                menu_wrap_index(&avoid_index, AVOID_PARAM_COUNT);
            }
            else if (key == KEY_2)
            {
                avoid_index++;
                menu_wrap_index(&avoid_index, AVOID_PARAM_COUNT);
            }
            else if (key == KEY_3)
            {
                avoid_param_selected = true;
            }
            else if (key == KEY_4)
            {
                menu_page = MENU_PAGE_TOP;
                avoid_param_selected = false;
            }
        }
    }
    else if (menu_page == MENU_PAGE_TEST)
    {
        if (towpoint_selected)
        {
            if (key == KEY_1)
            {
                if (test_index == 0)
                    towpoint_set_up_row(towpoint_get_up_row() + TOWPOINT_STEP);
                else if (test_index == 1)
                    towpoint_set_down_row(towpoint_get_down_row() + TOWPOINT_STEP);
                else if (test_index == 2)
                    startline_set_stop_target(startline_get_stop_target() + STARTLINE_TARGET_STEP);
                else if (test_index == 3)
                    test_adjust_param(TEST_PARAM_YAW_PRINT, 1);
                  else if (test_index == 4)
                      avoid_set_enabled(true);
                  else if (test_index == 5)
                      beep_set_enabled(1);
                  else
                      image_transfer_set_enabled(true);
            }
            else if (key == KEY_2)
            {
                if (test_index == 0)
                    towpoint_set_up_row(towpoint_get_up_row() - TOWPOINT_STEP);
                else if (test_index == 1)
                    towpoint_set_down_row(towpoint_get_down_row() - TOWPOINT_STEP);
                else if (test_index == 2)
                    startline_set_stop_target(startline_get_stop_target() - STARTLINE_TARGET_STEP);
                else if (test_index == 3)
                    test_adjust_param(TEST_PARAM_YAW_PRINT, -1);
                  else if (test_index == 4)
                      avoid_set_enabled(false);
                  else if (test_index == 5)
                      beep_set_enabled(0);
                  else
                      image_transfer_set_enabled(false);
            }
            else if (key == KEY_4)
            {
                MyFlash_SaveParameters();
                towpoint_selected = false;
                  printf("[MENU] test saved tow=%d/%d startline=%d yaw=%d avoid=%d beep=%d img=%d\r\n",
                         towpoint_get_up_row(), towpoint_get_down_row(), startline_get_stop_target(),
                         test_get_yaw_print_enabled() ? 1 : 0, avoid_is_enabled() ? 1 : 0,
                         beep_is_enabled() ? 1 : 0, image_transfer_is_enabled() ? 1 : 0);
            }
        }
        else
        {
            if (key == KEY_1)
            {
                test_index--;
                menu_wrap_index(&test_index, TEST_ITEM_COUNT);
            }
            else if (key == KEY_2)
            {
                test_index++;
                menu_wrap_index(&test_index, TEST_ITEM_COUNT);
            }
            else if (key == KEY_3)
            {
                towpoint_selected = true;
            }
            else if (key == KEY_4)
            {
                menu_page = MENU_PAGE_TOP;
                towpoint_selected = false;
            }
        }
    }
}

void Menu_Draw(void)
{
    if (!menu_dirty)
        return;

    ips200.clear();
    menu_dirty = false;

    if (menu_page == MENU_PAGE_TOP)
    {
        ips200.show_string(0, 0, "MAIN MENU");
        ips200.show_string(0, 16, "K1 UP K2 DN");
        ips200.show_string(0, 32, top_index == 0 ? ">Speed" : " Speed");
        ips200.show_string(0, 48, top_index == 1 ? ">PID" : " PID");
        ips200.show_string(0, 64, top_index == 2 ? ">Avoid" : " Avoid");
        ips200.show_string(0, 80, top_index == 3 ? ">Test" : " Test");
        ips200.show_string(0, 104, "K3 IN K4 RUN");
    }
    else if (menu_page == MENU_PAGE_MOTOR)
    {
        ips200.show_string(0, 0, "SPEED MENU");
        ips200.show_string(0, 14, base_speed_selected ? "K1+ K2- K4SAVE" : "K1/K2 K3 SEL");
        for (int i = 0; i < MOTOR_ITEM_COUNT; i++)
        {
            int y = 28 + i * 13;
            ips200.show_string(8, y, motor_index == i ? ">" : " ");
            ips200.show_string(18, y, motor_get_speed_param_name(i));
            if (i == 0 || i == 6)
                ips200.show_int(104, y, (int)motor_get_speed_param_value(i), 3);
            else
                ips200.show_float(104, y, motor_get_speed_param_value(i), 1, 1);
        }
        if (base_speed_selected)
            draw_red_marker(0, 30 + motor_index * 13);
    }
    else if (menu_page == MENU_PAGE_PID)
    {
        ips200.show_string(0, 0, "PID MENU");
        ips200.show_string(0, 14, pid_param_selected ? "K1+ K2- K4SAVE" : "K1/K2 K3 SEL");
        for (int i = 0; i < PID_ITEM_COUNT; i++)
        {
            int y = 32 + i * 14;
            ips200.show_string(8, y, pid_index == i ? ">" : " ");
            ips200.show_string(18, y, motor_get_turn_param_name(i));
            ips200.show_float(100, y, motor_get_turn_param_value(i), 2, 1);
        }
        if (pid_param_selected)
            draw_red_marker(0, 34 + pid_index * 14);
    }
    else if (menu_page == MENU_PAGE_AVOID)
    {
        ips200.show_string(0, 0, "AVOID MENU");
        ips200.show_string(0, 16, avoid_param_selected ? "K1+ K2- K4SAVE" : "K1/K2 K3 SEL");
        ips200.show_string(12, 40, avoid_get_param_name(avoid_index));
        if (avoid_index == 6)
            ips200.show_int(104, 40, (int)avoid_get_param_value(avoid_index), 3);
        else
            ips200.show_float(104, 40, avoid_get_param_value(avoid_index), 3, 1);
        ips200.show_string(12, 64, "Item");
        ips200.show_int(64, 64, avoid_index + 1, 1);
        ips200.show_string(80, 64, "/");
        ips200.show_int(96, 64, AVOID_PARAM_COUNT, 1);
        if (avoid_param_selected)
            draw_red_marker(0, 42);
        ips200.show_string(0, 96, avoid_param_selected ? "Editing" : "K4 BACK");
    }
    else if (menu_page == MENU_PAGE_TEST)
    {
        ips200.show_string(0, 0, "TEST MENU");
        ips200.show_string(0, 16, towpoint_selected ? "K1+ K2- K4SAVE" : "K1/K2 K3 SEL");
        ips200.show_string(12, 32, test_index == 0 ? ">TowUp" : " TowUp");
        ips200.show_int(104, 32, towpoint_get_up_row(), 2);
        ips200.show_string(12, 46, test_index == 1 ? ">TowDown" : " TowDown");
        ips200.show_int(104, 46, towpoint_get_down_row(), 2);
        ips200.show_string(12, 60, test_index == 2 ? ">StopCnt" : " StopCnt");
        ips200.show_int(104, 60, startline_get_stop_target(), 2);
        ips200.show_string(12, 74, test_index == 3 ? ">YawPrn" : " YawPrn");
        ips200.show_int(104, 74, test_get_yaw_print_enabled() ? 1 : 0, 1);
        ips200.show_string(12, 88, test_index == 4 ? ">AvoidEn" : " AvoidEn");
        ips200.show_int(104, 88, avoid_is_enabled() ? 1 : 0, 1);
          ips200.show_string(12, 102, test_index == 5 ? ">BeepEn" : " BeepEn");
          ips200.show_int(104, 102, beep_is_enabled() ? 1 : 0, 1);
          ips200.show_string(12, 116, test_index == 6 ? ">ImgTx" : " ImgTx");
          ips200.show_int(104, 116, image_transfer_is_enabled() ? 1 : 0, 1);
          if (towpoint_selected)
              draw_red_marker(0, test_index == 0 ? 34 : (test_index == 1 ? 48 : (test_index == 2 ? 62 : (test_index == 3 ? 76 : (test_index == 4 ? 90 : (test_index == 5 ? 104 : 116))))));
    }
}
