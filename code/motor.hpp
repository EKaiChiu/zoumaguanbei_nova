#ifndef motor_hpp
#define motor_hpp

#include "zf_common_headfile.hpp"

#define motor1_dir  ZF_GPIO_MOTOR_1
#define motor2_dir  ZF_GPIO_MOTOR_2

#define motor1_pwm      ZF_PWM_MOTOR_1
#define motor2_pwm      ZF_PWM_MOTOR_2

#define encoderA  ZF_ENCODER_DIR_1
#define encoderB  ZF_ENCODER_DIR_2

extern zf_driver_pwm motor1_pwm_1;   // 添加这行
extern zf_driver_pwm motor2_pwm_2;   // 添加这行

void encoder_test();

void motor_argument();
void motor_init(void);
void motor_pid_left();
void motor_pid_right();
void motor_control();
void motor_diff_pid1();

#endif