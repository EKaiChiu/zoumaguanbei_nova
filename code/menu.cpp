#include "menu.hpp"
#include "Key.hpp"
#include "init.hpp"
#include "motor.hpp"

#define MENU_MODE_MIN 0
#define MENU_MODE_MAX 4
#define MENU_PWM_MIN 0
#define MENU_PWM_MAX 3500
#define MENU_PWM_STEP 100

typedef enum
{
    MENU_PAGE_MODE = 0,
    MENU_PAGE_MODE2_PWM
} MenuPage;

static MenuPage menu_page = MENU_PAGE_MODE;
static int selected_mode = 2;

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

void Menu_Process(void)
{
    Key_Process();
    int key = Key_GetValueOnce();

    if (key == KEY_NONE)
        return;

    if (menu_page == MENU_PAGE_MODE)
    {
        if (key == KEY_1)
        {
            selected_mode--;
            if (selected_mode < MENU_MODE_MIN)
                selected_mode = MENU_MODE_MAX;
        }
        else if (key == KEY_2)
        {
            selected_mode++;
            if (selected_mode > MENU_MODE_MAX)
                selected_mode = MENU_MODE_MIN;
        }
        else if (key == KEY_3)
        {
            selected_mode = test_mode;
        }
        else if (key == KEY_4)
        {
            test_mode = selected_mode;
            if (test_mode == 2)
                menu_page = MENU_PAGE_MODE2_PWM;
        }
    }
    else if (menu_page == MENU_PAGE_MODE2_PWM)
    {
        if (key == KEY_1)
        {
            mode2_pwm = clamp_int(mode2_pwm - MENU_PWM_STEP, MENU_PWM_MIN, MENU_PWM_MAX);
        }
        else if (key == KEY_2)
        {
            mode2_pwm = clamp_int(mode2_pwm + MENU_PWM_STEP, MENU_PWM_MIN, MENU_PWM_MAX);
        }
        else if (key == KEY_3)
        {
            menu_page = MENU_PAGE_MODE;
            selected_mode = test_mode;
        }
        else if (key == KEY_4)
        {
            test_mode = 2;
        }
    }
}

void Menu_Draw(void)
{
    if (menu_page == MENU_PAGE_MODE)
    {
        ips200.show_string(0, 0, "MODE MENU");
        ips200.show_string(0, 16, "K1-/K2+ K4 OK");
        ips200.show_string(0, 32, "Select:");
        ips200.show_int(64, 32, selected_mode, 1);
        ips200.show_string(0, 48, "Run:");
        ips200.show_int(40, 48, test_mode, 1);
        ips200.show_string(0, 64, "K3 reset");
    }
    else if (menu_page == MENU_PAGE_MODE2_PWM)
    {
        ips200.show_string(0, 0, "MODE2 PWM");
        ips200.show_string(0, 16, "K1-/K2+ K3 Back");
        ips200.show_string(0, 32, "PWM:");
        ips200.show_int(40, 32, mode2_pwm, 4);
        ips200.show_string(0, 48, "K4 Confirm");
    }
}
