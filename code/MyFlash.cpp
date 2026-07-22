#include "MyFlash.hpp"

#include "avoid.hpp"
#include "beep.hpp"

extern void image_transfer_set_enabled(bool enable);
extern bool image_transfer_is_enabled(void);
#include "ips200_draw.hpp"
#include "motor.hpp"
#include "StartLine.hpp"
#include "Test.hpp"

#include "zf_common_headfile.hpp"
#include <stdio.h>
#include <string.h>

static const char *FLASH_PARAM_FILE = "nova_params.txt";
static const char *FLASH_KEY_LINE_BASE_SPEED = "line_base_speed=";
static const char *FLASH_KEY_TOWPOINT_UP = "towpoint_up_row=";
static const char *FLASH_KEY_TOWPOINT_DOWN = "towpoint_down_row=";
static const char *FLASH_KEY_STARTLINE_STOP_TARGET = "startline_stop_target=";
static const char *FLASH_KEY_TEST_YAW_PRINT = "test_yaw_print=";
static const char *FLASH_KEY_AVOID_ENABLED = "avoid_enabled=";
static const char *FLASH_KEY_BEEP_ENABLED = "beep_enabled=";
static const char *FLASH_KEY_IMAGE_TRANSFER_ENABLED = "image_transfer_enabled=";
static const char *FLASH_KEY_SPEED_EXPECT_LIMIT = "speed_expect_limit=";
static const char *FLASH_TURN_KEYS[MOTOR_TURN_PARAM_COUNT] = {
    "turn_low_kp=",
    "turn_mid_kp=",
    "turn_big_kp=",
    "turn_sharp_kp=",
    "turn_ring_mul="};
static const char *FLASH_SPEED_RATIO_KEYS[5] = {
    "speed_ring_ratio=",
    "speed_small_ratio=",
    "speed_mid_ratio=",
    "speed_big_ratio=",
    "speed_sharp_ratio="};
static const char *FLASH_AVOID_KEYS[AVOID_PARAM_COUNT] = {
    "avoid_right_max=",
    "avoid_left_max=",
    "avoid_shift_step=",
    "avoid_return_step=",
    "avoid_hold_time="};

static zf_driver_file_string flash_file(FLASH_PARAM_FILE, "a+");
static bool flash_ready = false;
static bool flash_loaded_line_base_speed = false;
static bool flash_loaded_towpoint_up = false;
static bool flash_loaded_towpoint_down = false;
static bool flash_loaded_startline_stop_target = false;
static bool flash_loaded_test_yaw_print = false;
static bool flash_loaded_avoid_enabled = false;
static bool flash_loaded_beep_enabled = false;
static bool flash_loaded_image_transfer_enabled = false;
static bool flash_loaded_avoid_param[AVOID_PARAM_COUNT] = {false};
static bool flash_loaded_speed_ratio[5] = {false};
static bool flash_loaded_speed_expect_limit = false;
static bool flash_loaded_turn_param[MOTOR_TURN_PARAM_COUNT] = {false};

static bool parse_int_key(const char *text, const char *key, int *value)
{
    if (text == NULL || key == NULL || value == NULL)
        return false;

    int key_len = (int)strlen(key);
    if (strncmp(text, key, key_len) != 0)
        return false;

    int parsed = 0;
    if (sscanf(text + key_len, "%d", &parsed) != 1)
        return false;

    *value = parsed;
    return true;
}

static bool parse_line_base_speed(const char *text, int *value)
{
    return parse_int_key(text, FLASH_KEY_LINE_BASE_SPEED, value);
}

static bool parse_avoid_param(const char *text, int *index, float *value)
{
    if (text == NULL || index == NULL || value == NULL)
        return false;

    for (int i = 0; i < AVOID_PARAM_COUNT; i++)
    {
        int key_len = (int)strlen(FLASH_AVOID_KEYS[i]);
        if (strncmp(text, FLASH_AVOID_KEYS[i], key_len) == 0)
        {
            float parsed = 0.0f;
            if (sscanf(text + key_len, "%f", &parsed) != 1)
                return false;
            *index = i;
            *value = parsed;
            return true;
        }
    }

    return false;
}

static bool parse_speed_ratio(const char *text, int *index, float *value)
{
    if (text == NULL || index == NULL || value == NULL)
        return false;

    for (int i = 0; i < 5; i++)
    {
        int key_len = (int)strlen(FLASH_SPEED_RATIO_KEYS[i]);
        if (strncmp(text, FLASH_SPEED_RATIO_KEYS[i], key_len) == 0)
        {
            float parsed = 0.0f;
            if (sscanf(text + key_len, "%f", &parsed) != 1)
                return false;
            *index = i;
            *value = parsed;
            return true;
        }
    }

    return false;
}

static bool parse_turn_param(const char *text, int *index, float *value)
{
    if (text == NULL || index == NULL || value == NULL)
        return false;

    for (int i = 0; i < MOTOR_TURN_PARAM_COUNT; i++)
    {
        int key_len = (int)strlen(FLASH_TURN_KEYS[i]);
        if (strncmp(text, FLASH_TURN_KEYS[i], key_len) == 0)
        {
            float parsed = 0.0f;
            if (sscanf(text + key_len, "%f", &parsed) != 1)
                return false;
            *index = i;
            *value = parsed;
            return true;
        }
    }

    return false;
}

static bool all_avoid_params_loaded(void)
{
    for (int i = 0; i < AVOID_PARAM_COUNT; i++)
    {
        if (!flash_loaded_avoid_param[i])
            return false;
    }
    return true;
}

static bool all_speed_ratios_loaded(void)
{
    for (int i = 0; i < 5; i++)
    {
        if (!flash_loaded_speed_ratio[i])
            return false;
    }
    return true;
}

static bool all_turn_params_loaded(void)
{
    for (int i = 0; i < MOTOR_TURN_PARAM_COUNT; i++)
    {
        if (!flash_loaded_turn_param[i])
            return false;
    }
    return true;
}

void MyFlash_Init(void)
{
    flash_file.set_path(FLASH_PARAM_FILE, "a+");
    flash_file.rewind_file();
    flash_ready = true;

    MyFlash_LoadParameters();
    if (!flash_loaded_line_base_speed || !all_speed_ratios_loaded() || !flash_loaded_speed_expect_limit ||
        !all_turn_params_loaded() || !all_avoid_params_loaded() || !flash_loaded_avoid_enabled ||
        !flash_loaded_towpoint_up || !flash_loaded_towpoint_down ||
        !flash_loaded_startline_stop_target || !flash_loaded_test_yaw_print || !flash_loaded_beep_enabled ||
        !flash_loaded_image_transfer_enabled)
    {
        MyFlash_SaveParameters();
    }

    printf("[FLASH] ready: %s\r\n", FLASH_PARAM_FILE);
}

void MyFlash_LoadParameters(void)
{
    if (!flash_ready)
    {
        flash_file.set_path(FLASH_PARAM_FILE, "a+");
        flash_ready = true;
    }

    flash_file.rewind_file();

    char item[96] = {0};
    flash_loaded_line_base_speed = false;
    flash_loaded_towpoint_up = false;
    flash_loaded_towpoint_down = false;
    flash_loaded_startline_stop_target = false;
    flash_loaded_test_yaw_print = false;
    flash_loaded_avoid_enabled = false;
    flash_loaded_beep_enabled = false;
    flash_loaded_image_transfer_enabled = false;
    flash_loaded_speed_expect_limit = false;
    for (int i = 0; i < AVOID_PARAM_COUNT; i++)
        flash_loaded_avoid_param[i] = false;
    for (int i = 0; i < 5; i++)
        flash_loaded_speed_ratio[i] = false;
    for (int i = 0; i < MOTOR_TURN_PARAM_COUNT; i++)
        flash_loaded_turn_param[i] = false;

    while (flash_file.read_string(item) == 0)
    {
        int int_value = 0;
        if (parse_line_base_speed(item, &int_value))
        {
            motor_set_line_base_speed(int_value);
            flash_loaded_line_base_speed = true;
            printf("[FLASH] load line_base_speed=%d\r\n", motor_get_line_base_speed());
            continue;
        }

        int speed_index = 0;
        float speed_value = 0.0f;
        if (parse_speed_ratio(item, &speed_index, &speed_value))
        {
            motor_set_speed_param_value(speed_index + 1, speed_value);
            flash_loaded_speed_ratio[speed_index] = true;
            printf("[FLASH] load %s=%.1f\r\n", motor_get_speed_param_name(speed_index + 1),
                   motor_get_speed_param_value(speed_index + 1));
            continue;
        }

        if (parse_int_key(item, FLASH_KEY_SPEED_EXPECT_LIMIT, &int_value))
        {
            motor_set_speed_param_value(6, (float)int_value);
            flash_loaded_speed_expect_limit = true;
            printf("[FLASH] load %s=%d\r\n", motor_get_speed_param_name(6),
                   (int)motor_get_speed_param_value(6));
            continue;
        }

        int turn_index = 0;
        float turn_value = 0.0f;
        if (parse_turn_param(item, &turn_index, &turn_value))
        {
            motor_set_turn_param_value(turn_index, turn_value);
            flash_loaded_turn_param[turn_index] = true;
            printf("[FLASH] load %s=%.1f\r\n", motor_get_turn_param_name(turn_index),
                   motor_get_turn_param_value(turn_index));
            continue;
        }

        if (parse_int_key(item, FLASH_KEY_TOWPOINT_UP, &int_value))
        {
            towpoint_set_up_row(int_value);
            flash_loaded_towpoint_up = true;
            printf("[FLASH] load towpoint_up_row=%d\r\n", towpoint_get_up_row());
            continue;
        }

        if (parse_int_key(item, FLASH_KEY_TOWPOINT_DOWN, &int_value))
        {
            towpoint_set_down_row(int_value);
            flash_loaded_towpoint_down = true;
            printf("[FLASH] load towpoint_down_row=%d\r\n", towpoint_get_down_row());
            continue;
        }

        if (parse_int_key(item, FLASH_KEY_STARTLINE_STOP_TARGET, &int_value))
        {
            startline_set_stop_target(int_value);
            flash_loaded_startline_stop_target = true;
            printf("[FLASH] load startline_stop_target=%d\r\n", startline_get_stop_target());
            continue;
        }

        if (parse_int_key(item, FLASH_KEY_TEST_YAW_PRINT, &int_value))
        {
            test_set_yaw_print_enabled(int_value != 0);
            flash_loaded_test_yaw_print = true;
            printf("[FLASH] load test_yaw_print=%d\r\n", test_get_yaw_print_enabled() ? 1 : 0);
            continue;
        }

        if (parse_int_key(item, FLASH_KEY_AVOID_ENABLED, &int_value))
        {
            avoid_set_enabled(int_value != 0);
            flash_loaded_avoid_enabled = true;
            printf("[FLASH] load avoid_enabled=%d\r\n", avoid_is_enabled() ? 1 : 0);
            continue;
        }

        if (parse_int_key(item, FLASH_KEY_BEEP_ENABLED, &int_value))
        {
            beep_set_enabled(int_value != 0);
            flash_loaded_beep_enabled = true;
            printf("[FLASH] load beep_enabled=%d\r\n", beep_is_enabled() ? 1 : 0);
            continue;
        }

        if (parse_int_key(item, FLASH_KEY_IMAGE_TRANSFER_ENABLED, &int_value))
        {
            image_transfer_set_enabled(int_value != 0);
            flash_loaded_image_transfer_enabled = true;
            printf("[FLASH] load image_transfer_enabled=%d\r\n", image_transfer_is_enabled() ? 1 : 0);
            continue;
        }

        int avoid_index = 0;
        float avoid_value = 0.0f;
        if (parse_avoid_param(item, &avoid_index, &avoid_value))
        {
            avoid_set_param_value(avoid_index, avoid_value);
            flash_loaded_avoid_param[avoid_index] = true;
            printf("[FLASH] load %s=%.1f\r\n", avoid_get_param_name(avoid_index), avoid_get_param_value(avoid_index));
        }
    }

    if (!flash_loaded_line_base_speed)
        printf("[FLASH] line_base_speed default=%d\r\n", motor_get_line_base_speed());
    if (!all_speed_ratios_loaded())
        printf("[FLASH] speed ratio default ring=%.1f small=%.1f mid=%.1f big=%.1f sharp=%.1f\r\n",
               motor_get_speed_param_value(1), motor_get_speed_param_value(2),
               motor_get_speed_param_value(3), motor_get_speed_param_value(4),
               motor_get_speed_param_value(5));
    if (!flash_loaded_speed_expect_limit)
        printf("[FLASH] speed limit default=%d\r\n", (int)motor_get_speed_param_value(6));
    if (!all_turn_params_loaded())
        printf("[FLASH] turn kp default low=%.1f mid=%.1f big=%.1f sharp=%.1f ring=%.1f\r\n",
               motor_get_turn_param_value(0), motor_get_turn_param_value(1),
               motor_get_turn_param_value(2), motor_get_turn_param_value(3),
               motor_get_turn_param_value(4));
    if (!flash_loaded_towpoint_up || !flash_loaded_towpoint_down)
        printf("[FLASH] towpoint default up=%d down=%d\r\n", towpoint_get_up_row(), towpoint_get_down_row());
    if (!flash_loaded_startline_stop_target)
        printf("[FLASH] startline_stop_target default=%d\r\n", startline_get_stop_target());
    if (!flash_loaded_test_yaw_print)
        printf("[FLASH] test_yaw_print default=%d\r\n", test_get_yaw_print_enabled() ? 1 : 0);
    if (!flash_loaded_avoid_enabled)
        printf("[FLASH] avoid_enabled default=%d\r\n", avoid_is_enabled() ? 1 : 0);
    if (!flash_loaded_beep_enabled)
        printf("[FLASH] beep_enabled default=%d\r\n", beep_is_enabled() ? 1 : 0);
    if (!flash_loaded_image_transfer_enabled)
        printf("[FLASH] image_transfer_enabled default=%d\r\n", image_transfer_is_enabled() ? 1 : 0);
}

void MyFlash_SaveParameters(void)
{
    if (!flash_ready)
    {
        flash_file.set_path(FLASH_PARAM_FILE, "a+");
        flash_ready = true;
    }

    flash_file.set_path(FLASH_PARAM_FILE, "w+");
    flash_file.rewind_file();

    char item[96] = {0};
    bool save_ok = true;

    snprintf(item, sizeof(item), "%s%d\r\n", FLASH_KEY_LINE_BASE_SPEED, motor_get_line_base_speed());
    if (flash_file.write_string(item) != 0)
        save_ok = false;
    else
        flash_loaded_line_base_speed = true;

    for (int i = 0; i < 5; i++)
    {
        snprintf(item, sizeof(item), "%s%.1f\r\n", FLASH_SPEED_RATIO_KEYS[i], motor_get_speed_param_value(i + 1));
        if (flash_file.write_string(item) != 0)
            save_ok = false;
        else
            flash_loaded_speed_ratio[i] = true;
    }

    snprintf(item, sizeof(item), "%s%d\r\n", FLASH_KEY_SPEED_EXPECT_LIMIT,
             (int)motor_get_speed_param_value(6));
    if (flash_file.write_string(item) != 0)
        save_ok = false;
    else
        flash_loaded_speed_expect_limit = true;

    for (int i = 0; i < MOTOR_TURN_PARAM_COUNT; i++)
    {
        snprintf(item, sizeof(item), "%s%.1f\r\n", FLASH_TURN_KEYS[i], motor_get_turn_param_value(i));
        if (flash_file.write_string(item) != 0)
            save_ok = false;
        else
            flash_loaded_turn_param[i] = true;
    }

    for (int i = 0; i < AVOID_PARAM_COUNT; i++)
    {
        snprintf(item, sizeof(item), "%s%.1f\r\n", FLASH_AVOID_KEYS[i], avoid_get_param_value(i));
        if (flash_file.write_string(item) != 0)
            save_ok = false;
        else
            flash_loaded_avoid_param[i] = true;
    }

    snprintf(item, sizeof(item), "%s%d\r\n", FLASH_KEY_TOWPOINT_UP, towpoint_get_up_row());
    if (flash_file.write_string(item) != 0)
        save_ok = false;
    else
        flash_loaded_towpoint_up = true;

    snprintf(item, sizeof(item), "%s%d\r\n", FLASH_KEY_TOWPOINT_DOWN, towpoint_get_down_row());
    if (flash_file.write_string(item) != 0)
        save_ok = false;
    else
        flash_loaded_towpoint_down = true;

    snprintf(item, sizeof(item), "%s%d\r\n", FLASH_KEY_STARTLINE_STOP_TARGET, startline_get_stop_target());
    if (flash_file.write_string(item) != 0)
        save_ok = false;
    else
        flash_loaded_startline_stop_target = true;

    snprintf(item, sizeof(item), "%s%d\r\n", FLASH_KEY_TEST_YAW_PRINT, test_get_yaw_print_enabled() ? 1 : 0);
    if (flash_file.write_string(item) != 0)
        save_ok = false;
    else
        flash_loaded_test_yaw_print = true;

    snprintf(item, sizeof(item), "%s%d\r\n", FLASH_KEY_AVOID_ENABLED, avoid_is_enabled() ? 1 : 0);
    if (flash_file.write_string(item) != 0)
        save_ok = false;
    else
        flash_loaded_avoid_enabled = true;

    snprintf(item, sizeof(item), "%s%d\r\n", FLASH_KEY_BEEP_ENABLED, beep_is_enabled() ? 1 : 0);
    if (flash_file.write_string(item) != 0)
        save_ok = false;
    else
        flash_loaded_beep_enabled = true;

    snprintf(item, sizeof(item), "%s%d\r\n", FLASH_KEY_IMAGE_TRANSFER_ENABLED, image_transfer_is_enabled() ? 1 : 0);
    if (flash_file.write_string(item) != 0)
        save_ok = false;
    else
        flash_loaded_image_transfer_enabled = true;

    if (save_ok)
    {
        printf("[FLASH] save base=%d limit=%d speed/pid ok avoid ok tow=%d/%d startline=%d yaw=%d avoid=%d beep=%d img=%d\r\n",
               motor_get_line_base_speed(), (int)motor_get_speed_param_value(6),
               towpoint_get_up_row(), towpoint_get_down_row(), startline_get_stop_target(),
               test_get_yaw_print_enabled() ? 1 : 0, avoid_is_enabled() ? 1 : 0,
               beep_is_enabled() ? 1 : 0, image_transfer_is_enabled() ? 1 : 0);
    }
    else
    {
        printf("[FLASH] save failed: %s\r\n", FLASH_PARAM_FILE);
    }
}
