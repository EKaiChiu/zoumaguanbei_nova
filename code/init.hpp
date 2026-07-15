#ifndef init_hpp
#define init_hpp

#include "zf_common_headfile.hpp"

extern zf_device_uvc uvc_cam;
extern zf_device_ips200 ips200;
extern zf_device_imu imu_dev;
extern zf_driver_pit pit_timer;
extern zf_driver_pit avoid_timer;
extern zf_driver_gpio beep_gpio;



extern uint8 beep;
extern volatile int car_start_flag;
/*****************�ⲿ����������*****************/
void init_all();
void start_motor_timer();  // 🛡️ 首帧后启动电机控制
void start_car();
void stop_car();
void stop_motor_timer();


#endif /* init_hpp */
