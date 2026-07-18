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

extern int test_mode;
extern int mode2_pwm;
extern int16 diff_speedl_expect;
extern int16 diff_speedr_expect;

void encoder_test();

void motor_argument();
void motor_init(void);
void motor_pid_left();
void motor_pid_right();
void motor_control();
void motor_oscilloscope_send();
void motor_mode6_set_lr_params(float kp_l, float ki_l, float kd_l, float ff_l, int min_l, float kp_r, float ki_r, float kd_r, float ff_r, int min_r, int target_speed);
void motor_mode6_adjust_target(int direction);
int motor_mode6_get_target(void);
void mode5_set_manual_params(float kp, float ki, float kd, float feedforward, int min_pwm, int target_speed);
void mode5_set_manual_lr_params(float kp_l, float ki_l, float kd_l, float ff_l, int min_l, float kp_r, float ki_r, float kd_r, float ff_r, int min_r, int target_speed);
void motor_diff_pid1();
void motor_set_line_base_speed(int speed);
int motor_get_line_base_speed(void);

#define MOTOR_SPEED_PARAM_COUNT 7
#define MOTOR_TURN_PARAM_COUNT 5

const char *motor_get_speed_param_name(int index);
float motor_get_speed_param_value(int index);
void motor_set_speed_param_value(int index, float value);
void motor_adjust_speed_param(int index, int direction);

const char *motor_get_turn_param_name(int index);
float motor_get_turn_param_value(int index);
void motor_set_turn_param_value(int index, float value);
void motor_adjust_turn_param(int index, int direction);

#endif
