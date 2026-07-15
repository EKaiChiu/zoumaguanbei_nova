#include "MyFlash.hpp"

#include "avoid.hpp"
#include "ips200_draw.hpp"
#include "motor.hpp"

#include "zf_common_headfile.hpp"
#include <stdio.h>
#include <string.h>

static const char *FLASH_PARAM_FILE = "nova_params.txt";
static const char *FLASH_KEY_LINE_BASE_SPEED = "line_base_speed=";
static const char *FLASH_KEY_TOWPOINT_UP = "towpoint_up_row=";
static const char *FLASH_KEY_TOWPOINT_DOWN = "towpoint_down_row=";
static const char *FLASH_AVOID_KEYS[AVOID_PARAM_COUNT] = {
    "avoid_right_max=",
    "avoid_left_max=",
    "avoid_ring_right_max=",
    "avoid_ring_left_max=",
    "avoid_shift_step=",
    "avoid_return_step=",
    "avoid_hold_time="};

static zf_driver_file_string flash_file(FLASH_PARAM_FILE, "a+");
static bool flash_ready = false;
static bool flash_loaded_line_base_speed = false;
static bool flash_loaded_towpoint_up = false;
static bool flash_loaded_towpoint_down = false;
static bool flash_loaded_avoid_param[AVOID_PARAM_COUNT] = {false};

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

static bool all_avoid_params_loaded(void)
{
    for (int i = 0; i < AVOID_PARAM_COUNT; i++)
    {
        if (!flash_loaded_avoid_param[i])
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
    if (!flash_loaded_line_base_speed || !all_avoid_params_loaded() ||
        !flash_loaded_towpoint_up || !flash_loaded_towpoint_down)
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
    for (int i = 0; i < AVOID_PARAM_COUNT; i++)
        flash_loaded_avoid_param[i] = false;

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
    if (!flash_loaded_towpoint_up || !flash_loaded_towpoint_down)
        printf("[FLASH] towpoint default up=%d down=%d\r\n", towpoint_get_up_row(), towpoint_get_down_row());
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

    if (save_ok)
    {
        printf("[FLASH] save base=%d avoid ok tow=%d/%d\r\n",
               motor_get_line_base_speed(), towpoint_get_up_row(), towpoint_get_down_row());
    }
    else
    {
        printf("[FLASH] save failed: %s\r\n", FLASH_PARAM_FILE);
    }
}
