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
    // 左绕行状态
    AVOID_TURN_LEFT_45,
    AVOID_TURN_RIGHT_TO_ZERO,
    AVOID_STRAIGHT_AFTER_ZERO,
    AVOID_TURN_RIGHT_TO_MINUS_45,
    // 右绕行状态
    AVOID_TURN_RIGHT_45,
    AVOID_TURN_LEFT_TO_ZERO,
    AVOID_TURN_LEFT_TO_45,
};

static const float AVOID_CONTROL_DT_S = 0.02f;
//左转参数
static const float AVOID_LEFT_TARGET_DEG = 30.0f;//拐出角度
static const float AVOID_ZERO_TARGET_DEG = 0.0f;//恢复角度
static const float AVOID_RIGHT_TARGET_DEG = -30.0f;
static const int AVOID_STRAIGHT_TICKS = 25;
static const int16 AVOID_TURN_LEFT_SPEED_L = -160;
static const int16 AVOID_TURN_LEFT_SPEED_R = 160;
static const int16 AVOID_TURN_RIGHT_SPEED_L = 160;
static const int16 AVOID_TURN_RIGHT_SPEED_R = -160;
static const int16 AVOID_STRAIGHT_SPEED = 160;

static AvoidState avoid_state = AVOID_DISABLED;
static int latest_vision_result = -1;
static float avoid_angle_deg = 0.0f;
static int avoid_straight_ticks = 0;
static int printed_state = -1;

static void avoid_print_state_once(void)
{
    if (printed_state != (int)avoid_state)
    {
        printed_state = (int)avoid_state;
        printf("[AVOID] state=%d angle=%.1f\r\n", (int)avoid_state, avoid_angle_deg);
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

void avoid_init(void)
{
    avoid_state = AVOID_DISABLED;
    latest_vision_result = -1;
    avoid_angle_deg = 0.0f;
    avoid_straight_ticks = 0;
    printed_state = -1;
}

void avoid_set_enabled(bool enable)
{
    avoid_state = enable ? AVOID_IDLE : AVOID_DISABLED;
    latest_vision_result = -1;
    avoid_angle_deg = 0.0f;
    avoid_straight_ticks = 0;
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

void avoid_force_start(void)
{
    if (avoid_state == AVOID_DISABLED)
        avoid_state = AVOID_IDLE;

    avoid_state = AVOID_TURN_LEFT_45;
    latest_vision_result = -1;
    avoid_angle_deg = 0.0f;
    avoid_straight_ticks = 0;
    printed_state = -1;
}

bool avoid_control_left(void)
{
    if (avoid_state == AVOID_DISABLED)
    {
        latest_vision_result = -1;
        return false;
    }

    if (avoid_state == AVOID_IDLE && avoid_is_left_result(latest_vision_result))
    {
        avoid_state = AVOID_TURN_LEFT_45;
        latest_vision_result = -1;
        avoid_angle_deg = 0.0f;
        avoid_straight_ticks = 0;
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

        if (fabsf(avoid_angle_deg) < AVOID_LEFT_TARGET_DEG)
        {
            diff_speedl_expect = AVOID_TURN_LEFT_SPEED_L;
            diff_speedr_expect = AVOID_TURN_LEFT_SPEED_R;
            return true;
        }

        avoid_state = AVOID_TURN_RIGHT_TO_ZERO;
        printed_state = -1;
        diff_speedl_expect = AVOID_TURN_RIGHT_SPEED_L;
        diff_speedr_expect = AVOID_TURN_RIGHT_SPEED_R;
        return true;
    }

    if (avoid_state == AVOID_TURN_RIGHT_TO_ZERO)
    {
        avoid_angle_deg += get_gyro_z_dps(imu_dev) * AVOID_CONTROL_DT_S;
        if (avoid_angle_deg > AVOID_ZERO_TARGET_DEG)
        {
            diff_speedl_expect = AVOID_TURN_RIGHT_SPEED_L;
            diff_speedr_expect = AVOID_TURN_RIGHT_SPEED_R;
            return true;
        }

        avoid_state = AVOID_STRAIGHT_AFTER_ZERO;
        avoid_straight_ticks = 0;
        printed_state = -1;
        diff_speedl_expect = AVOID_STRAIGHT_SPEED;
        diff_speedr_expect = AVOID_STRAIGHT_SPEED;
        return true;
    }

    if (avoid_state == AVOID_STRAIGHT_AFTER_ZERO)
    {
        avoid_angle_deg += get_gyro_z_dps(imu_dev) * AVOID_CONTROL_DT_S;
        avoid_straight_ticks++;
        diff_speedl_expect = AVOID_STRAIGHT_SPEED;
        diff_speedr_expect = AVOID_STRAIGHT_SPEED;

        if (avoid_straight_ticks >= AVOID_STRAIGHT_TICKS)
        {
            avoid_state = AVOID_TURN_RIGHT_TO_MINUS_45;
            printed_state = -1;
        }
        return true;
    }

    if (avoid_state == AVOID_TURN_RIGHT_TO_MINUS_45)
    {
        avoid_angle_deg += get_gyro_z_dps(imu_dev) * AVOID_CONTROL_DT_S;

        if (avoid_angle_deg > AVOID_RIGHT_TARGET_DEG)
        {
            diff_speedl_expect = AVOID_TURN_RIGHT_SPEED_L;
            diff_speedr_expect = AVOID_TURN_RIGHT_SPEED_R;
            return true;
        }

        avoid_state = AVOID_IDLE;
        latest_vision_result = -1;
        avoid_angle_deg = 0.0f;
        avoid_straight_ticks = 0;
        printed_state = -1;
        return false;
    }

    return false;
}

bool avoid_control_right(void)
{
    if (avoid_state == AVOID_DISABLED)
    {
        latest_vision_result = -1;
        return false;
    }

    if (avoid_state == AVOID_IDLE && avoid_is_right_result(latest_vision_result))
    {
        avoid_state = AVOID_TURN_RIGHT_45;
        latest_vision_result = -1;
        avoid_angle_deg = 0.0f;
        avoid_straight_ticks = 0;
        printed_state = -1;
    }

    if (avoid_state == AVOID_IDLE)
    {
        return false;
    }

    avoid_print_state_once();

    // 阶段①：右转 30°（车头朝右）
    if (avoid_state == AVOID_TURN_RIGHT_45)
    {
        avoid_angle_deg += get_gyro_z_dps(imu_dev) * AVOID_CONTROL_DT_S;

        if (avoid_angle_deg > AVOID_RIGHT_TARGET_DEG)
        {
            diff_speedl_expect = AVOID_TURN_RIGHT_SPEED_L;
            diff_speedr_expect = AVOID_TURN_RIGHT_SPEED_R;
            return true;
        }

        avoid_state = AVOID_TURN_LEFT_TO_ZERO;
        printed_state = -1;
        diff_speedl_expect = AVOID_TURN_LEFT_SPEED_L;
        diff_speedr_expect = AVOID_TURN_LEFT_SPEED_R;
        return true;
    }

    // 阶段②：左转回 0°（车头回正）
    if (avoid_state == AVOID_TURN_LEFT_TO_ZERO)
    {
        avoid_angle_deg += get_gyro_z_dps(imu_dev) * AVOID_CONTROL_DT_S;
        if (avoid_angle_deg < AVOID_ZERO_TARGET_DEG)
        {
            diff_speedl_expect = AVOID_TURN_LEFT_SPEED_L;
            diff_speedr_expect = AVOID_TURN_LEFT_SPEED_R;
            return true;
        }

        avoid_state = AVOID_STRAIGHT_AFTER_ZERO;
        avoid_straight_ticks = 0;
        printed_state = -1;
        diff_speedl_expect = AVOID_STRAIGHT_SPEED;
        diff_speedr_expect = AVOID_STRAIGHT_SPEED;
        return true;
    }

    // 阶段③：直行（与左绕行共用）
    if (avoid_state == AVOID_STRAIGHT_AFTER_ZERO)
    {
        avoid_angle_deg += get_gyro_z_dps(imu_dev) * AVOID_CONTROL_DT_S;
        avoid_straight_ticks++;
        diff_speedl_expect = AVOID_STRAIGHT_SPEED;
        diff_speedr_expect = AVOID_STRAIGHT_SPEED;

        if (avoid_straight_ticks >= AVOID_STRAIGHT_TICKS)
        {
            avoid_state = AVOID_TURN_LEFT_TO_45;
            printed_state = -1;
        }
        return true;
    }

    // 阶段④：左转到 +30°（最终航向）
    if (avoid_state == AVOID_TURN_LEFT_TO_45)
    {
        avoid_angle_deg += get_gyro_z_dps(imu_dev) * AVOID_CONTROL_DT_S;

        if (avoid_angle_deg < AVOID_LEFT_TARGET_DEG)
        {
            diff_speedl_expect = AVOID_TURN_LEFT_SPEED_L;
            diff_speedr_expect = AVOID_TURN_LEFT_SPEED_R;
            return true;
        }

        avoid_state = AVOID_IDLE;
        latest_vision_result = -1;
        avoid_angle_deg = 0.0f;
        avoid_straight_ticks = 0;
        printed_state = -1;
        return false;
    }

    return false;
}

int avoid_get_state(void)
{
    return (int)avoid_state;
}
