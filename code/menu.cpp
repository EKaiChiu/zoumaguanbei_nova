#include "menu.hpp"
#include "Key.hpp"
#include "init.hpp"
#include "motor.hpp"
#include "MyFlash.hpp"
#include "avoid.hpp"
#include "ips200_draw.hpp"

#define MENU_TOP_COUNT 3
#define MOTOR_ITEM_COUNT 1
#define TEST_ITEM_COUNT 2
#define BASE_SPEED_STEP 5
#define TOWPOINT_STEP 1

// 一级菜单：Motor / Avoid
// 二级菜单：Motor 里目前只有 BaseSpeed；Avoid 暂时为空。
typedef enum
{
    MENU_PAGE_TOP = 0,
    MENU_PAGE_MOTOR,
    MENU_PAGE_AVOID,
    MENU_PAGE_TEST
} MenuPage;

static MenuPage menu_page = MENU_PAGE_TOP;
static int top_index = 0;
static int motor_index = 0;
static bool base_speed_selected = false;
static int avoid_index = 0;
static bool avoid_param_selected = false;
static int test_index = 0;
static bool towpoint_selected = false;

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

void Menu_Process(void)
{
    Key_Process();
    int key = Key_GetValueOnce();

    if (key == KEY_NONE)
        return;

    if (car_start_flag)
    {
        if (key == KEY_4)
        {
            stop_car();
            menu_page = MENU_PAGE_TOP;
            base_speed_selected = false;
            avoid_param_selected = false;
            towpoint_selected = false;
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
                menu_page = MENU_PAGE_AVOID;
            else
                menu_page = MENU_PAGE_TEST;
            motor_index = 0;
            avoid_index = 0;
            test_index = 0;
            base_speed_selected = false;
            avoid_param_selected = false;
            towpoint_selected = false;
        }
        else if (key == KEY_4)
        {
            printf("[MENU] KEY4 launch\r\n");
            start_car();
        }
    }
    else if (menu_page == MENU_PAGE_MOTOR)
    {
        if (base_speed_selected)
        {
            if (key == KEY_1)
            {
                motor_set_line_base_speed(motor_get_line_base_speed() + BASE_SPEED_STEP);
            }
            else if (key == KEY_2)
            {
                motor_set_line_base_speed(motor_get_line_base_speed() - BASE_SPEED_STEP);
            }
            else if (key == KEY_4)
            {
                MyFlash_SaveParameters();
                base_speed_selected = false;
                printf("[MENU] base speed saved=%d\r\n", motor_get_line_base_speed());
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
                if (motor_index == 0)
                    base_speed_selected = true;
            }
            else if (key == KEY_4)
            {
                menu_page = MENU_PAGE_TOP;
                base_speed_selected = false;
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
                else
                    towpoint_set_down_row(towpoint_get_down_row() + TOWPOINT_STEP);
            }
            else if (key == KEY_2)
            {
                if (test_index == 0)
                    towpoint_set_up_row(towpoint_get_up_row() - TOWPOINT_STEP);
                else
                    towpoint_set_down_row(towpoint_get_down_row() - TOWPOINT_STEP);
            }
            else if (key == KEY_4)
            {
                MyFlash_SaveParameters();
                towpoint_selected = false;
                printf("[MENU] towpoint up=%d down=%d saved\r\n", towpoint_get_up_row(), towpoint_get_down_row());
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
    if (menu_page == MENU_PAGE_TOP)
    {
        ips200.show_string(0, 0, "MAIN MENU");
        ips200.show_string(0, 16, "K1 UP K2 DN");
        ips200.show_string(0, 32, top_index == 0 ? ">Motor" : " Motor");
        ips200.show_string(0, 48, top_index == 1 ? ">Avoid" : " Avoid");
        ips200.show_string(0, 64, top_index == 2 ? ">Test" : " Test");
        ips200.show_string(0, 96, "K3 IN K4 RUN");
    }
    else if (menu_page == MENU_PAGE_MOTOR)
    {
        ips200.show_string(0, 0, "MOTOR MENU");
        ips200.show_string(0, 16, base_speed_selected ? "K1+ K2- K4SAVE" : "K1/K2 K3 SEL");
        ips200.show_string(12, 40, motor_index == 0 ? ">BaseSpeed" : " BaseSpeed");
        ips200.show_int(104, 40, motor_get_line_base_speed(), 3);
        if (base_speed_selected)
            draw_red_marker(0, 42);
        ips200.show_string(0, 80, base_speed_selected ? "Editing..." : "K4 BACK");
    }
    else if (menu_page == MENU_PAGE_AVOID)
    {
        ips200.show_string(0, 0, "AVOID MENU");
        ips200.show_string(0, 16, avoid_param_selected ? "K1+ K2- K4OK" : "K1/K2 K3 SEL");
        ips200.show_string(12, 40, avoid_get_param_name(avoid_index));
        ips200.show_float(88, 40, avoid_get_param_value(avoid_index), 3, 1);
        if (avoid_param_selected)
            draw_red_marker(0, 42);
        ips200.show_string(0, 80, avoid_param_selected ? "Editing..." : "K4 BACK");
    }
    else if (menu_page == MENU_PAGE_TEST)
    {
        ips200.show_string(0, 0, "TEST MENU");
        ips200.show_string(0, 16, towpoint_selected ? "K1+ K2- K4SAVE" : "K1/K2 K3 SEL");
        ips200.show_string(12, 40, test_index == 0 ? ">TowUp" : " TowUp");
        ips200.show_int(88, 40, towpoint_get_up_row(), 2);
        ips200.show_string(12, 56, test_index == 1 ? ">TowDown" : " TowDown");
        ips200.show_int(88, 56, towpoint_get_down_row(), 2);
        if (towpoint_selected)
            draw_red_marker(0, test_index == 0 ? 42 : 58);
        ips200.show_string(0, 88, towpoint_selected ? "Editing..." : "K4 BACK");
    }
}
