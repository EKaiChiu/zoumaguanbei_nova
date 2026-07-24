#include "Voltage.hpp"
#include "motor.hpp"

// 这里使用 motor.cpp 里已经创建好的电机方向 GPIO 和 PWM 对象。
// 本文件不会主动输出，只有外部显式调用 voltage_set_percent() 才会改变电机口输出。
extern zf_driver_gpio motor1_gpio;
extern zf_driver_gpio motor2_gpio;

static int voltage_left_percent = 0;
static int voltage_right_percent = 0;
static bool voltage_enabled = false;

static int voltage_clamp_percent(int percent)
{
    if (percent < 0)
        return 0;
    if (percent > 100)
        return 100;
    return percent;
}

int voltage_percent_to_duty(int percent)
{
    percent = voltage_clamp_percent(percent);
    return (VOLTAGE_DUTY_MAX * percent) / 100;
}

void voltage_set_percent(int channel, int percent)
{
    percent = voltage_clamp_percent(percent);

    // 电压输出已禁用：只记录参数，不再实际驱动电机 PWM 口。
    if (channel == VOLTAGE_CHANNEL_LEFT)
        voltage_left_percent = percent;
    else if (channel == VOLTAGE_CHANNEL_RIGHT)
        voltage_right_percent = percent;
}

void voltage_set_enabled(bool enabled)
{
    voltage_enabled = enabled;
}

bool voltage_is_enabled(void)
{
    return voltage_enabled;
}

void voltage_set_left_percent(int percent)
{
    voltage_left_percent = voltage_clamp_percent(percent);
}

int voltage_get_left_percent(void)
{
    return voltage_left_percent;
}

void voltage_apply_left_output(void)
{
    // 电压输出已禁用，保留接口避免菜单/flash 编译受影响。
}

void voltage_stop_left_output(void)
{
    // 电压输出已禁用，普通电机停止仍由 stop_car()/motor_control() 管理。
}

void voltage_stop(int channel)
{
    voltage_set_percent(channel, 0);
}

int voltage_get_last_percent(int channel)
{
    if (channel == VOLTAGE_CHANNEL_LEFT)
        return voltage_left_percent;
    if (channel == VOLTAGE_CHANNEL_RIGHT)
        return voltage_right_percent;
    return 0;
}
