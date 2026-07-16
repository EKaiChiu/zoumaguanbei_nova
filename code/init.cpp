#include "init.hpp"
#include "avoid.hpp"
#include "imu660.hpp"
#include "motor.hpp"
#include "MyFlash.hpp"
#include "StartLine.hpp"
#include "Test.hpp"


uint8 beep=0;
#define beep_pin ZF_GPIO_BEEP
zf_device_uvc uvc_cam;
zf_device_ips200 ips200;
zf_device_imu imu_dev;           // IMU660RA 陀螺仪
zf_driver_pit pit_timer;
zf_driver_pit avoid_timer;
zf_driver_gpio beep_gpio(beep_pin,O_RDWR);

// 图像就绪标志：防止定时器在图像未准备好时就控制电机
volatile int image_ready_flag = 0;
// 发车标志：后续接按键后再置 1，避免程序启动后自动发车
volatile int car_start_flag = 0;

void Interrupt()//中断函数
{
  if(!image_ready_flag) return;  // 🛡️ 图像没就绪时不执行电机控制！
  if(!car_start_flag) return;    // 未发车时不执行电机控制
  imu_yaw_print_task(imu_dev, 0.02f);  // IMU yaw debug task, 20ms period
  test_update(imu_dev, 0.02f);          // Test菜单调试任务
  motor_control();         // 核心：每5ms执行一次差速和电机PID
}


void AvoidInterrupt()
{
  if(!image_ready_flag) return;
  if(!car_start_flag) return;
  avoid_update_control();
}

// ⭐ 手动启动定时器（在main.cpp中第一帧处理后调用）
void start_motor_timer()
{
    image_ready_flag = 1;
    printf("[SYSTEM] image ready, wait launch flag.\r\n");
}

void start_car()
{
    if (car_start_flag)
        return;

    startline_reset();
    car_start_flag = 1;
    printf("[SYSTEM] car launch enabled. startline reset target=%d\r\n", startline_get_stop_target());
}

void stop_car()
{
    if (!car_start_flag)
        return;

    car_start_flag = 0;
    motor1_pwm_1.set_duty(0);
    motor2_pwm_2.set_duty(0);
    printf("[SYSTEM] car launch disabled.\r\n");
}

void stop_motor_timer()
{
    image_ready_flag = 0;
    car_start_flag = 0;
    motor1_pwm_1.set_duty(0);
    motor2_pwm_2.set_duty(0);
    printf("[SYSTEM] motor control stopped\r\n");
}

void init_all()
{
    system_delay_ms(500);
    MyFlash_Init();

    beep_gpio.set_level(beep);
    ips200.init(FB_PATH);
   
   
    if(uvc_cam.init(UVC_PATH)<0)
    {ips200.show_string(0,20,"UVC init error");}

    // ---- 陀螺仪 IMU660RA 初始化 + 零偏校准 ----
    if(imu_dev.init() != DEV_NO_FIND)
    {
        printf("[IMU] Device found, calibrating gyro offset...\r\n");
        system_delay_ms(100);               // 等陀螺仪稳定
        gyroOffsetInit(imu_dev);            // 静止采样100次，获取零偏
        quaternion_init();                  // 姿态四元数复位（当前朝向=0°）
        printf("[IMU] Calibration done, ready.\r\n");
    }
    else
    {
        printf("[IMU] Device not found, skip.\r\n");
    }

    motor_init();          // 电机 PWM 初始化
    avoid_init();
    avoid_set_enabled(AVOID_MODE != 0);
    test_init();
    
     motor_argument();      // 🌟 必须取消注释！给目标速度和 PID 参数赋值
    
    ips200.clear();
    ips200.show_string(116, 112, "Fantasy_Sim");
    system_delay_ms(1000);
    ips200.clear();
    
    // ⚠️ 定时器在这里初始化但先不启动！
    // 由 main.cpp 在第一帧图像处理完后调用 start_motor_timer()
    image_ready_flag = 0;  // 先禁用电机控制
    car_start_flag = 0;    // 先不发车
    pit_timer.init_ms(20, Interrupt);  // 注册中断回调（但flag=0不会执行）
    avoid_timer.init_ms(5, AvoidInterrupt); // 绕行状态机专用中断
}
