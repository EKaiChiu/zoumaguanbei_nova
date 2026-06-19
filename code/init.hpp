#ifndef init_hpp
#define init_hpp

#include "zf_common_headfile.hpp"

extern zf_device_uvc uvc_cam;
extern zf_device_ips200 ips200;
extern zf_driver_pit pit_timer;



extern uint8 beep;
/*****************�ⲿ����������*****************/
void init_all();
void start_motor_timer();  // 🛡️ 首帧后启动电机控制


#endif /* init_hpp */