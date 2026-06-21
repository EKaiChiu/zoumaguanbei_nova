#include "zf_common_headfile.hpp"
#include "zf_device_imu.hpp"
#include "imu660.hpp"

/*
 * test.cpp  ——  IMU yaw 角变动打印测试
 *
 * 用途：验证 IMU660RA 陀螺仪 + 姿态解算是否正常工作。
 *       转动小车，串口实时输出 yaw 角变化。
 *
 * 使用方式：
 *   1. 将 main.cpp 临时改名（如 main.cpp.bak）
 *   2. 编译运行此文件
 *   3. 用手转动小车，观察串口输出
 */

zf_device_imu imu_dev;

int main()
{
    // ---- 1. IMU 初始化 ----
    printf("=== IMU Yaw Test ===\r\n");

    if (imu_dev.init() == DEV_NO_FIND)
    {
        printf("[ERROR] IMU not found!\r\n");
        while (1);
    }

    printf("[INFO] Calibrating gyro, keep still...\r\n");
    system_delay_ms(100);
    gyroOffsetInit(imu_dev);
    quaternion_init();
    printf("[INFO] Calibration done. Rotate the car to see yaw change.\r\n\r\n");

    // ---- 2. 主循环：1ms 积分 + 打印 yaw 变动 ----
    float last_yaw = 0.0f;

    while (1)
    {
        // 模拟 1ms 定时中断：更新四元数 + 转欧拉角
        quaternion_update(imu_dev);
        quaternion_to_euler();

        float yaw = get_yaw();

        // 只在 yaw 变动超过 0.5° 时打印（过滤静止噪声）
        if (fabsf(yaw - last_yaw) > 0.5f)
        {
            printf("yaw: %.1f°  (delta: %+.1f°)\r\n", yaw, yaw - last_yaw);
            last_yaw = yaw;
        }

        system_delay_ms(1);  // 1ms 周期
    }

    return 0;
}
