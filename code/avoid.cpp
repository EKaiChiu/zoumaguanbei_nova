#include "avoid.hpp"

#include "imu660.hpp"
#include "init.hpp"
#include "motor.hpp"

#include <math.h>
#include <stdio.h>

enum AvoidState
{
    AVOID_DISABLED = 0,
    AVOID_IDLE,
    AVOID_TURN_LEFT_45,
};

static const float AVOID_CONTROL_DT_S = 0.02f;
static const float AVOID_TARGET_ANGLE_DEG = 45.0f;
static const int16 AVOID_LEFT_SPEED = -60;
static const int16 AVOID_RIGHT_SPEED = 60;

static AvoidState avoid_state = AVOID_IDLE;
static int latest_vision_result = -1;
static float avoid_angle_deg = 0.0f;
static int printed_state = -1;

static void avoid_print_state_once(void)
{
    if (printed_state != (int)avoid_state)
    {
        printed_state = (int)avoid_state;
        printf("[AVOID] state=%d angle=%.1f\r\n", (int)avoid_state, avoid_angle_deg);
    }
}

static bool avoid_is_target_result(int result)
{
    return result == 0 || result == 1 || result == 2;
}

void avoid_init(void)
{
    avoid_state = AVOID_IDLE;
    latest_vision_result = -1;
    avoid_angle_deg = 0.0f;
    printed_state = -1;
}

void avoid_set_enabled(bool enable)
{
    avoid_state = enable ? AVOID_IDLE : AVOID_DISABLED;
    latest_vision_result = -1;
    avoid_angle_deg = 0.0f;
    printed_state = -1;
    printf("[AVOID] %s\r\n", enable ? "enabled" : "disabled");
}

bool avoid_is_enabled(void)
{
    return avoid_state != AVOID_DISABLED;
}

void avoid_set_vision_result(int result)
{
    latest_vision_result = result;
}

bool avoid_control(void)
{
    if (avoid_state == AVOID_DISABLED)
    {
        latest_vision_result = -1;
        return false;
    }

    if (avoid_state == AVOID_IDLE && avoid_is_target_result(latest_vision_result))
    {
        avoid_state = AVOID_TURN_LEFT_45;
        latest_vision_result = -1;
        avoid_angle_deg = 0.0f;
        printed_state = -1;
    }

    if (avoid_state == AVOID_IDLE)
    {
        return false;
    }

    avoid_print_state_once();

    if (avoid_state == AVOID_TURN_LEFT_45)
    {
        avoid_angle_deg += get_gyro_z_dps(imu_dev) * AVOID_CONTROL_DT_S;

        if (fabsf(avoid_angle_deg) < AVOID_TARGET_ANGLE_DEG)
        {
            diff_speedl_expect = AVOID_LEFT_SPEED;
            diff_speedr_expect = AVOID_RIGHT_SPEED;
            return true;
        }

        avoid_state = AVOID_IDLE;
        latest_vision_result = -1;
        avoid_angle_deg = 0.0f;
        printed_state = -1;
        return false;
    }

    return false;
}

int avoid_get_state(void)
{
    return (int)avoid_state;
}
