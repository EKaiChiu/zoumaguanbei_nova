#include "imu660.hpp"
#include <math.h>
#include <stdio.h>

#ifndef PI
#define PI 3.14159265358979f
#endif
// ==================== 全局变量定义 ====================
Quaternion quat = {1.0f, 0.0f, 0.0f, 0.0f};   // 姿态四元数，姿态更新的核心状态量
EulerAngle euler = {0.0f, 0.0f, 0.0f};          // 欧拉角输出（度），由 quaternion_to_euler 刷新
gyro_param_t GyroOffset = {0.0f, 0.0f, 0.0f};   // 陀螺仪静止零偏，校准后用于实时扣除
uint8 gyro_offset_flag = 0;                      // 零偏校准完成标志：0=未校准, 1=已校准

// ==================== 姿态解算核心 ====================

/*
 * gyroOffsetInit  ——  零偏校准（每次上电必须执行一次）
 *
 * 用途：陀螺仪上电后存在固定的输出偏置（零漂），不校准会导致积分角度持续漂移。
 *       此函数在静止状态下采样100次角速度取均值，写入全局 GyroOffset，
 *       后续 gyroOffsetInit 和 get_gyro_*_dps 自动扣除该偏置。
 *
 * 调用时机：imu_dev.init() 成功后、小车保持绝对静止时调用一次。
 *          耗时约 500ms（100次 × 5ms）。
 *
 * 依赖：调用前小车必须静止，否则校准值不准确。
 * 产出：gyro_offset_flag 置 1，GyroOffset 有效。
 */
void gyroOffsetInit(zf_device_imu &imu_dev)
{
    gyro_offset_flag = 0;

    GyroOffset.Xdata = 0;
    GyroOffset.Ydata = 0;
    GyroOffset.Zdata = 0;

    for (uint16_t i = 0; i < 100; i++)
    {
        GyroOffset.Xdata += imu_dev.get_gyro_x();
        GyroOffset.Ydata += imu_dev.get_gyro_y();
        GyroOffset.Zdata += imu_dev.get_gyro_z();
        system_delay_ms(5);
    }

    GyroOffset.Xdata /= 100;
    GyroOffset.Ydata /= 100;
    GyroOffset.Zdata /= 100;
    gyro_offset_flag = 1;
}

/*
 * quaternion_init  ——  姿态复位
 *
 * 用途：将四元数重置为单位四元数（对应欧拉角 0°），
 *       相当于把当前朝向定义为"正前方"。
 *
 * 调用时机：
 *   - 系统初始化时（配合 gyroOffsetInit 使用）
 *   - 需要重新设定朝向零点时（例如发车前手动对准）
 *
 * 注意：只重置四元数，不重置 GyroOffset（零偏保持不变）。
 */
void quaternion_init(void)
{
    quat.q0 = 1.0f;
    quat.q1 = 0.0f;
    quat.q2 = 0.0f;
    quat.q3 = 0.0f;
}

/*
 * quaternion_update  ——  姿态积分（高频核心，每 1ms 调用）
 *
 * 用途：这是姿态解算的主循环。读取陀螺仪角速度 → 扣除零偏 → 数值积分
 *       → 更新四元数。四元数是后续转欧拉角的输入。
 *
 * 调用时机：必须在 1ms 定时中断中调用，保证积分步长 dt=0.001s 准确。
 *          通常放在 pit_timer 的 ISR 里。
 *
 * 处理流程：
 *   1. 读陀螺仪 raw → 减 GyroOffset → 转 dps → 转 rad/s
 *   2. 死区滤波：|角速度| < 0.005 rad/s 置零（抑制静止噪声积分）
 *   3. 一阶龙格库塔法积分四元数
 *   4. 四元数归一化（消除数值漂移）
 *
 * 依赖：gyroOffsetInit 必须先执行完成（gyro_offset_flag == 1）。
 * 产出：更新全局 quat，供 quaternion_to_euler 消费。
 */
void quaternion_update(zf_device_imu &imu_dev)
{
    // Step 1: 读取原始值、去零偏、转换为 dps → rad/s
    float gx_raw = imu660ra_gyro_transition(imu_dev.get_gyro_x() - (int16)GyroOffset.Xdata) * PI / 180.0f;
    float gy_raw = imu660ra_gyro_transition(imu_dev.get_gyro_y() - (int16)GyroOffset.Ydata) * PI / 180.0f;
    float gz_raw = imu660ra_gyro_transition(imu_dev.get_gyro_z() - (int16)GyroOffset.Zdata) * PI / 180.0f;

    // Step 2: 死区滤波 —— 小于 0.005 rad/s（≈ 0.3°/s）的角速度视为噪声，置零
    if (fabsf(gx_raw) < 0.005f) gx_raw = 0;
    if (fabsf(gy_raw) < 0.005f) gy_raw = 0;
    if (fabsf(gz_raw) < 0.005f) gz_raw = 0;

    float dt = 0.001f;  // 1ms 积分步长，与调用周期严格对应

    // Step 3: 四元数微分方程（一阶龙格库塔法 / 欧拉法）
    float q0 = quat.q0;
    float q1 = quat.q1;
    float q2 = quat.q2;
    float q3 = quat.q3;

    quat.q0 += (-q1 * gx_raw - q2 * gy_raw - q3 * gz_raw) * 0.5f * dt;
    quat.q1 += ( q0 * gx_raw + q2 * gz_raw - q3 * gy_raw) * 0.5f * dt;
    quat.q2 += ( q0 * gy_raw - q1 * gz_raw + q3 * gx_raw) * 0.5f * dt;
    quat.q3 += ( q0 * gz_raw + q1 * gy_raw - q2 * gx_raw) * 0.5f * dt;

    // Step 4: 四元数归一化 —— 防止多次积分后模长偏离 1
    float norm = sqrtf(quat.q0 * quat.q0 + quat.q1 * quat.q1 +
                       quat.q2 * quat.q2 + quat.q3 * quat.q3);
    quat.q0 /= norm;
    quat.q1 /= norm;
    quat.q2 /= norm;
    quat.q3 /= norm;
}

/*
 * quaternion_to_euler  ——  四元数 → 欧拉角转换
 *
 * 用途：将 quaternion_update 积分出的四元数转为直观的欧拉角（度），
 *       存入全局 euler（roll/pitch/yaw），供控制逻辑直接读取。
 *
 * 调用时机：紧跟 quaternion_update 之后调用（同样在 1ms 定时中断中）。
 *          调用频率应一致，否则欧拉角刷新滞后。
 *
 * 三轴含义（右手系，IMU 水平安装在小车上）：
 *   roll  — 绕前进轴旋转（小车左右倾斜）
 *   pitch — 绕横向轴旋转（小车前后倾斜/坡道）
 *   yaw   — 绕垂直轴旋转（小车水平转向/偏航角）
 *
 * 产出：euler.roll, euler.pitch, euler.yaw（单位：度）
 */
void quaternion_to_euler(void)
{
    euler.roll  = atan2f(2.0f * (quat.q0 * quat.q1 + quat.q2 * quat.q3),
                         1.0f - 2.0f * (quat.q1 * quat.q1 + quat.q2 * quat.q2));
    euler.pitch = asinf(2.0f * (quat.q0 * quat.q2 - quat.q3 * quat.q1));
    euler.yaw   = atan2f(2.0f * (quat.q0 * quat.q3 + quat.q1 * quat.q2),
                         1.0f - 2.0f * (quat.q2 * quat.q2 + quat.q3 * quat.q3));

    euler.roll  *= 180.0f / PI;
    euler.pitch *= 180.0f / PI;
    euler.yaw   *= 180.0f / PI;
}

// ==================== 数据获取接口 ====================

/*
 * get_gyro_*_dps  ——  获取实时角速度（去零偏，dps 单位）
 *
 * 用途：不经过四元数积分，直接读取当前的瞬时角速度。
 *      可用于：
 *        - 转弯时的实时角速度反馈（辅助差速控制）
 *        - 检测小车是否在旋转（陀螺仪弯道检测）
 *        - 调试打印陀螺仪数据
 *
 * 注意：内部自动扣除 GyroOffset，无需手动处理零偏。
 *       返回的是瞬时值，不是积分结果。需要角度用 get_yaw()。
 */
float get_gyro_x_dps(zf_device_imu &imu_dev)
{
    return imu660ra_gyro_transition(imu_dev.get_gyro_x() - (int16)GyroOffset.Xdata);
}
float get_gyro_y_dps(zf_device_imu &imu_dev)
{
    return imu660ra_gyro_transition(imu_dev.get_gyro_y() - (int16)GyroOffset.Ydata);
}
float get_gyro_z_dps(zf_device_imu &imu_dev)
{
    return imu660ra_gyro_transition(imu_dev.get_gyro_z() - (int16)GyroOffset.Zdata);
}

/*
 * get_acc_*_g  ——  获取加速度（g 单位，1g = 重力加速度）
 *
 * 用途：读取加速度计数据，可用于：
 *        - Z 轴：静止时约为 1g，用于检测坡道（倾角变化）
 *        - X/Y 轴：检测横向/纵向加速度，识别碰撞或急加速
 *        - 配合陀螺仪做互补滤波获得更稳定的 pitch/roll
 *
 * 注意：加速度计包含重力分量 + 运动加速度，单独使用时注意区分。
 */
float get_acc_x_g(zf_device_imu &imu_dev)
{
    return imu660ra_acc_transition(imu_dev.get_acc_x());
}
float get_acc_y_g(zf_device_imu &imu_dev)
{
    return imu660ra_acc_transition(imu_dev.get_acc_y());
}
float get_acc_z_g(zf_device_imu &imu_dev)
{
    return imu660ra_acc_transition(imu_dev.get_acc_z());
}

/*
 * get_roll / get_pitch / get_yaw  ——  欧拉角快捷读取
 *
 * 用途：一行代码获取当前欧拉角（度），免去直接访问 euler 结构体。
 *      数据由上一帧 quaternion_to_euler 刷新。
 *
 * 典型用法：
 *   if (get_yaw() > 90.0f) { ... }  // 判断已转过 90°
 *   float slope = get_pitch();       // 读取当前坡道角度
 */
float get_roll(void)  { return euler.roll; }
float get_pitch(void) { return euler.pitch; }
float get_yaw(void)   { return euler.yaw; }

// ==================== Yaw 实时打印状态机 ====================

enum ImuYawPrintState
{
    IMU_YAW_WAIT_OFFSET = 0, // 等待陀螺仪零偏校准完成
    IMU_YAW_INIT,            // 初始化：把本次调试的相对 yaw 清零
    IMU_YAW_RUN,             // 运行：持续积分 Z 轴角速度并打印 yaw
};

static ImuYawPrintState imu_yaw_print_state = IMU_YAW_WAIT_OFFSET;
static float imu_yaw_print_deg = 0.0f;
static int imu_yaw_print_count = 0;

// 关闭 yaw 调试打印：状态机仍会积分 yaw，但不再刷终端。
static const bool IMU_YAW_PRINT_ENABLE = false;

// 每 5 次 20ms 中断打印一次，也就是约 100ms 一次。
// 如果想每个控制周期都打印，把这个值改成 1。
static const int IMU_YAW_PRINT_INTERVAL = 5;

static float imu_yaw_wrap_180(float yaw)
{
    while (yaw > 180.0f)
    {
        yaw -= 360.0f;
    }
    while (yaw < -180.0f)
    {
        yaw += 360.0f;
    }
    return yaw;
}

void imu_yaw_print_reset(void)
{
    imu_yaw_print_state = IMU_YAW_INIT;
    imu_yaw_print_deg = 0.0f;
    imu_yaw_print_count = 0;
}

void imu_yaw_print_task(zf_device_imu &imu_dev, float dt_s)
{
    if (!gyro_offset_flag)
    {
        if (imu_yaw_print_state != IMU_YAW_WAIT_OFFSET)
        {
            if (IMU_YAW_PRINT_ENABLE)
            {
                printf("[IMU_YAW] wait gyro offset\r\n");
            }
        }
        imu_yaw_print_state = IMU_YAW_WAIT_OFFSET;
        return;
    }

    if (imu_yaw_print_state == IMU_YAW_WAIT_OFFSET || imu_yaw_print_state == IMU_YAW_INIT)
    {
        imu_yaw_print_deg = 0.0f;
        imu_yaw_print_count = 0;
        imu_yaw_print_state = IMU_YAW_RUN;
        if (IMU_YAW_PRINT_ENABLE)
        {
            printf("[IMU_YAW] start yaw=0.0\r\n");
        }
    }

    if (imu_yaw_print_state == IMU_YAW_RUN)
    {
        float gyro_z_dps = get_gyro_z_dps(imu_dev);
        imu_yaw_print_deg = imu_yaw_wrap_180(imu_yaw_print_deg + gyro_z_dps * dt_s);

        imu_yaw_print_count++;
        if (IMU_YAW_PRINT_ENABLE && imu_yaw_print_count >= IMU_YAW_PRINT_INTERVAL)
        {
            imu_yaw_print_count = 0;
            printf("[IMU_YAW] yaw=%.1f gz=%.1f\r\n", imu_yaw_print_deg, gyro_z_dps);
        }
    }
}

float imu_get_integrated_yaw(void)
{
    return imu_yaw_print_deg;
}
