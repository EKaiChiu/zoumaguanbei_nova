#include "avoid.hpp"

#include "image.hpp"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

enum AvoidState
{
    AVOID_DISABLED = -1,
    AVOID_IDLE = 0,
    AVOID_SHIFT,
    AVOID_HOLD,
    AVOID_RETURN,
    AVOID_LOCK,
};

static AvoidState avoid_state = AVOID_DISABLED;
static int latest_vision_result = -1;
static int avoid_dir = 0;
static float avoid_bias = 0.0f;
static uint16_t avoid_cnt = 0;
static int printed_state = -100;
static bool avoid_on_ring_latched = false;

float Trans_line = 0.0f;

static float avoid_right_max = 30.0f;
static float avoid_left_max = 30.0f;
static float avoid_ring_right_max = 12.0f;
static float avoid_ring_left_max = 12.0f;
static float avoid_shift_step = 3.0f;
static float avoid_return_step = 2.0f;
static uint16_t avoid_hold_time = 50; // 50 * 5ms = 250ms

static float clamp_float_local(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

const char *avoid_get_param_name(int index)
{
    static const char *names[AVOID_PARAM_COUNT] = {
        "RightMax", "LeftMax", "RingR", "RingL", "Shift", "Return", "Hold"};

    if (index < 0 || index >= AVOID_PARAM_COUNT)
        return "Unknown";
    return names[index];
}

float avoid_get_param_value(int index)
{
    switch (index)
    {
        case 0: return avoid_right_max;
        case 1: return avoid_left_max;
        case 2: return avoid_ring_right_max;
        case 3: return avoid_ring_left_max;
        case 4: return avoid_shift_step;
        case 5: return avoid_return_step;
        case 6: return (float)avoid_hold_time;
        default: return 0.0f;
    }
}

void avoid_set_param_value(int index, float value)
{
    switch (index)
    {
        case 0:
            avoid_right_max = clamp_float_local(value, 0.0f, 80.0f);
            break;
        case 1:
            avoid_left_max = clamp_float_local(value, 0.0f, 80.0f);
            break;
        case 2:
            avoid_ring_right_max = clamp_float_local(value, 0.0f, 50.0f);
            break;
        case 3:
            avoid_ring_left_max = clamp_float_local(value, 0.0f, 50.0f);
            break;
        case 4:
            avoid_shift_step = clamp_float_local(value, 0.5f, 20.0f);
            break;
        case 5:
            avoid_return_step = clamp_float_local(value, 0.5f, 20.0f);
            break;
        case 6:
        {
            int hold = (int)(value + 0.5f);
            if (hold < 0)
                hold = 0;
            if (hold > 500)
                hold = 500;
            avoid_hold_time = (uint16_t)hold;
            break;
        }
        default:
            break;
    }
}

void avoid_adjust_param(int index, int direction)
{
    if (direction == 0)
        return;

    float dir = direction > 0 ? 1.0f : -1.0f;
    switch (index)
    {
        case 0:
            avoid_right_max = clamp_float_local(avoid_right_max + dir * 1.0f, 0.0f, 80.0f);
            break;
        case 1:
            avoid_left_max = clamp_float_local(avoid_left_max + dir * 1.0f, 0.0f, 80.0f);
            break;
        case 2:
            avoid_ring_right_max = clamp_float_local(avoid_ring_right_max + dir * 1.0f, 0.0f, 50.0f);
            break;
        case 3:
            avoid_ring_left_max = clamp_float_local(avoid_ring_left_max + dir * 1.0f, 0.0f, 50.0f);
            break;
        case 4:
            avoid_shift_step = clamp_float_local(avoid_shift_step + dir * 0.5f, 0.5f, 20.0f);
            break;
        case 5:
            avoid_return_step = clamp_float_local(avoid_return_step + dir * 0.5f, 0.5f, 20.0f);
            break;
        case 6:
        {
            int value = (int)avoid_hold_time + (direction > 0 ? 5 : -5);
            if (value < 0)
                value = 0;
            if (value > 500)
                value = 500;
            avoid_hold_time = (uint16_t)value;
            break;
        }
        default:
            break;
    }
}

static bool avoid_is_left_result(int result)
{
    return result == 0;
}

static bool avoid_is_right_result(int result)
{
    return result == 1;
}

static bool avoid_has_target(void)
{
    return avoid_is_left_result(latest_vision_result) || avoid_is_right_result(latest_vision_result);
}

static bool avoid_is_ring_now(void)
{
    return ImageStatus.Road_type == LeftCirque || ImageStatus.Road_type == RightCirque ||
           ImageFlag.image_element_rings_flag != 0;
}

static float get_avoid_right_max(void)
{
    return avoid_on_ring_latched ? avoid_ring_right_max : avoid_right_max;
}

static float get_avoid_left_max(void)
{
    return avoid_on_ring_latched ? avoid_ring_left_max : avoid_left_max;
}

static float approach_float(float value, float target, float step)
{
    if (step < 0.1f)
        step = 0.1f;

    if (value < target)
    {
        value += step;
        if (value > target)
            value = target;
    }
    else if (value > target)
    {
        value -= step;
        if (value < target)
            value = target;
    }

    return value;
}

static void avoid_reset_runtime(AvoidState state)
{
    avoid_state = state;
    latest_vision_result = -1;
    avoid_dir = 0;
    avoid_bias = 0.0f;
    avoid_cnt = 0;
    avoid_on_ring_latched = false;
    Trans_line = 0.0f;
    printed_state = -100;
}

static void avoid_print_state_once(void)
{
    if (printed_state != (int)avoid_state)
    {
        printed_state = (int)avoid_state;
        printf("[AVOID] state=%d dir=%d bias=%.1f\r\n", (int)avoid_state, avoid_dir, avoid_bias);
    }
}

void avoid_init(void)
{
    avoid_reset_runtime(AVOID_DISABLED);
}

void avoid_set_enabled(bool enable)
{
    avoid_reset_runtime(enable ? AVOID_IDLE : AVOID_DISABLED);
    printf("[AVOID] %s\r\n", enable ? "enabled" : "disabled");
}

bool avoid_is_enabled(void)
{
    return avoid_state != AVOID_DISABLED;
}

void avoid_set_vision_result(int result)
{
    if (avoid_state == AVOID_DISABLED)
        return;

    latest_vision_result = result;
}

void avoid_force_start(void)
{
    if (avoid_state == AVOID_DISABLED)
        return;

    latest_vision_result = 0;
}

void avoid_update_control(void)
{
    float target_bias = 0.0f;

    switch (avoid_state)
    {
        case AVOID_DISABLED:
            avoid_bias = 0.0f;
            avoid_dir = 0;
            avoid_cnt = 0;
            avoid_on_ring_latched = false;
            break;

        case AVOID_IDLE:
            avoid_bias = 0.0f;
            avoid_dir = 0;
            avoid_cnt = 0;
            avoid_on_ring_latched = false;

            if (avoid_has_target())
            {
                avoid_on_ring_latched = avoid_is_ring_now();

                if (avoid_is_left_result(latest_vision_result))
                {
                    avoid_dir = 1; // 左绕：目标中线向右偏
                }
                else if (avoid_is_right_result(latest_vision_result))
                {
                    avoid_dir = -1; // 右绕：目标中线向左偏
                }

                avoid_state = AVOID_SHIFT;
                printed_state = -100;
            }
            break;

        case AVOID_SHIFT:
            target_bias = (avoid_dir < 0) ? -get_avoid_right_max() : get_avoid_left_max();
            avoid_bias = approach_float(avoid_bias, target_bias, avoid_shift_step);

            if (fabsf(avoid_bias - target_bias) < 0.5f)
            {
                avoid_cnt = 0;
                avoid_state = AVOID_HOLD;
                printed_state = -100;
            }
            break;

        case AVOID_HOLD:
            avoid_bias = (avoid_dir < 0) ? -get_avoid_right_max() : get_avoid_left_max();
            avoid_cnt++;

            if (avoid_cnt >= avoid_hold_time)
            {
                avoid_cnt = 0;
                avoid_state = AVOID_RETURN;
                printed_state = -100;
            }
            break;

        case AVOID_RETURN:
            avoid_bias = approach_float(avoid_bias, 0.0f, avoid_return_step);

            if (fabsf(avoid_bias) < 0.5f)
            {
                avoid_bias = 0.0f;
                avoid_dir = 0;
                avoid_cnt = 0;
                avoid_state = AVOID_LOCK;
                printed_state = -100;
            }
            break;

        case AVOID_LOCK:
            avoid_bias = 0.0f;
            avoid_dir = 0;

            if (!avoid_has_target())
            {
                latest_vision_result = -1;
                avoid_cnt = 0;
                avoid_on_ring_latched = false;
                avoid_state = AVOID_IDLE;
                printed_state = -100;
            }
            break;

        default:
            avoid_reset_runtime(AVOID_IDLE);
            break;
    }

    Trans_line = avoid_bias;
    avoid_print_state_once();
}

bool avoid_is_controlling(void)
{
    return false;
}

bool avoid_control_left(void)
{
    return false;
}

bool avoid_control_right(void)
{
    return false;
}

float avoid_get_trans_line(void)
{
    return Trans_line;
}

int avoid_get_state(void)
{
    return (int)avoid_state;
}
