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

static const float AVOID_RIGHT_MAX = 30.0f;
static const float AVOID_LEFT_MAX = 30.0f;
static const float AVOID_RING_RIGHT_MAX = 12.0f;
static const float AVOID_RING_LEFT_MAX = 12.0f;
static const float AVOID_SHIFT_STEP = 3.0f;
static const float AVOID_RETURN_STEP = 2.0f;
static const uint16_t AVOID_HOLD_TIME = 50; // 50 * 5ms = 250ms

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
    return avoid_on_ring_latched ? AVOID_RING_RIGHT_MAX : AVOID_RIGHT_MAX;
}

static float get_avoid_left_max(void)
{
    return avoid_on_ring_latched ? AVOID_RING_LEFT_MAX : AVOID_LEFT_MAX;
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
            avoid_bias = approach_float(avoid_bias, target_bias, AVOID_SHIFT_STEP);

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

            if (avoid_cnt >= AVOID_HOLD_TIME)
            {
                avoid_cnt = 0;
                avoid_state = AVOID_RETURN;
                printed_state = -100;
            }
            break;

        case AVOID_RETURN:
            avoid_bias = approach_float(avoid_bias, 0.0f, AVOID_RETURN_STEP);

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
