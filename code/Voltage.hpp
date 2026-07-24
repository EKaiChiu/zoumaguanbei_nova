#ifndef VOLTAGE_HPP
#define VOLTAGE_HPP

#include "zf_common_headfile.hpp"

// 电机输出口供电测试模块：只提供函数，不自动调用。
// channel: 1=左电机输出口，2=右电机输出口。
// percent: 0~100，对应 0%~100% PWM，占空比上限由 VOLTAGE_DUTY_MAX 决定。
#define VOLTAGE_CHANNEL_LEFT 1
#define VOLTAGE_CHANNEL_RIGHT 2
#define VOLTAGE_DUTY_MAX 3500

int voltage_percent_to_duty(int percent);
void voltage_set_percent(int channel, int percent);
void voltage_stop(int channel);
int voltage_get_last_percent(int channel);
void voltage_set_enabled(bool enabled);
bool voltage_is_enabled(void);
void voltage_set_left_percent(int percent);
int voltage_get_left_percent(void);
void voltage_apply_left_output(void);
void voltage_stop_left_output(void);

#endif
