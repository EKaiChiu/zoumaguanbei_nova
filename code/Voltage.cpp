#include "Voltage.hpp"
#include "motor.hpp"

// 这里使用 motor.cpp 里已经创建好的电机方向 GPIO 和 PWM 对象。
// 本文件不会主动输出，只有外部显式调用 voltage_set_percent() 才会改变电机口输出。
extern zf_driver_gpio motor1_gpio;
extern zf_driver_gpio motor2_gpio;

static int voltage_left_percent = 0;
static int voltage_right_percent = 0;

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
    int duty = voltage_percent_to_duty(percent);

    // 固定方向为 0，让电机口两线极性保持一致；需要反向时再单独改这里。
    if (channel == VOLTAGE_CHANNEL_LEFT)
    {
        voltage_left_percent = percent;
        motor1_gpio.set_level(0);
        motor1_pwm_1.set_duty((uint16)duty);
    }
    else if (channel == VOLTAGE_CHANNEL_RIGHT)
    {
        voltage_right_percent = percent;
        motor2_gpio.set_level(0);
        motor2_pwm_2.set_duty((uint16)duty);
    }
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
