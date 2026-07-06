#include "avoid.hpp"

#include "imu660.hpp"
#include "init.hpp"
#include "motor.hpp"

#include <math.h>
#include <stdio.h>

enum AvoidState
{
    AVOID_DISABLED = 0, /* 关闭：完全不接管电机，继续普通巡线 */
    AVOID_IDLE,         /* 空闲：绕行已开启，但还没有检测到目标 */
    AVOID_TURN_LEFT_45, /* 左转：检测到视觉结果 0 后，原地左转约 45 度 */
    AVOID_STOPPED,      /* 停车：转到目标角度后保持左右轮速度为 0 */
};

/* motor_control() 由 20ms PIT 调用一次，所以这里用 0.02s 积分角速度。 */
static const float AVOID_CONTROL_DT_S = 0.02f;

/* 示例目标角度：检测到 0 后左转 45 度。 */
static const float AVOID_TARGET_ANGLE_DEG = 45.0f;

/* 原地左转速度：左轮反转、右轮正转。方向反了就对调符号。 */
static const int16 AVOID_LEFT_SPEED = -60;
static const int16 AVOID_RIGHT_SPEED = 60;

/* 当前默认开启绕行功能，等待视觉结果 0 触发。 */
static AvoidState avoid_state = AVOID_IDLE;
static int latest_vision_result = -1;

/* 从进入左转状态开始累计的相对角度，单位：度。 */
static float avoid_angle_deg = 0.0f;
static int printed_state = -1;

static void avoid_print_state_once(void)
{
    /* 只在状态变化时打印一次，避免终端刷屏拖慢控制周期。 */
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
    /* main.cpp 中把 vision_get_from_rgb565() 的输出传进来。 */
    latest_vision_result = result;
}

bool avoid_control(void)
{
    if (avoid_state == AVOID_DISABLED)
    {
        latest_vision_result = -1;
        return false;
    }

    /* 触发条件：绕行开启，并且视觉识别输出为 0/1/2。 */
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
        /* 用陀螺仪 Z 轴角速度积分出相对转角。fabsf 只关心已转角度大小。 */
        avoid_angle_deg += get_gyro_z_dps(imu_dev) * AVOID_CONTROL_DT_S;

        if (fabsf(avoid_angle_deg) < AVOID_TARGET_ANGLE_DEG)
        {
            /* 还没到 45 度：继续原地左转。 */
            diff_speedl_expect = AVOID_LEFT_SPEED;
            diff_speedr_expect = AVOID_RIGHT_SPEED;
        }
        else
        {
            /* 到达目标角度：停车，并进入保持停车状态。 */
            diff_speedl_expect = 0;
            diff_speedr_expect = 0;
            avoid_state = AVOID_STOPPED;
            avoid_print_state_once();
        }

        return true;
    }

    if (avoid_state == AVOID_STOPPED)
    {
        /* 停车状态持续接管电机，防止回到巡线后又继续跑。 */
        diff_speedl_expect = 0;
        diff_speedr_expect = 0;
        return true;
    }

    return false;
}

int avoid_get_state(void)
{
    return (int)avoid_state;
}
