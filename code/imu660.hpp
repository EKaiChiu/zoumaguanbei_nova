#ifndef __IMU660_HPP__
#define __IMU660_HPP__

#include "zf_common_headfile.hpp"
#include "zf_device_imu.hpp"

/*********************************************************************************************************************
* 文件名称          imu660
* 功能说明          IMU660RA 陀螺仪姿态解算（四元数 + 欧拉角）
* 适用平台          LS2K0300
* 依赖库            zf_device_imu（逐飞官方IMU驱动）
*
* 修改记录
********************************************************************************************************************/

// ==================== 数据结构 ====================

// 四元数
typedef struct
{
    float q0, q1, q2, q3;
} Quaternion;

// 欧拉角（弧度/角度）
typedef struct
{
    float roll, pitch, yaw;
} EulerAngle;

// 陀螺仪零偏校准值
typedef struct
{
    float Xdata, Ydata, Zdata;
} gyro_param_t;

// ==================== 传感器原始值转换 ====================
/*
 * IMU660RA 陀螺仪量程 ±2000dps，分辨率 16.384 LSB/dps
 * raw → dps: raw / 16.384
 */
inline float imu660ra_gyro_transition(int16 raw)
{
    return (float)raw / 16.384f;
}

/*
 * IMU660RA 加速度计量程 ±16g，分辨率 2048 LSB/g
 * raw → g: raw / 2048.0
 */
inline float imu660ra_acc_transition(int16 raw)
{
    return (float)raw / 2048.0f;
}

// ==================== 全局变量声明 ====================
extern Quaternion quat;
extern EulerAngle euler;
extern gyro_param_t GyroOffset;
extern uint8 gyro_offset_flag;

// ==================== 函数声明 ====================

/*
函数名称：陀螺仪零偏校准
功能说明：采样100次静止角速度求均值，写入GyroOffset
参数说明：imu_dev - 已初始化的zf_device_imu对象引用
函数返回：无，完成后置位 gyro_offset_flag=1
修改时间：2026年6月21日
备    注：调用前需保持小车静止
example：
    gyroOffsetInit(imu_dev);
 */
void gyroOffsetInit(zf_device_imu &imu_dev);

/*
函数名称：四元数初始化
功能说明：重置四元数为单位四元数 (1,0,0,0)
example：
    quaternion_init();
 */
void quaternion_init(void);

/*
函数名称：四元数更新
功能说明：读取陀螺仪角速度，用一阶龙格库塔法更新四元数（1ms周期）
参数说明：imu_dev - 已初始化的zf_device_imu对象引用
example：
    quaternion_update(imu_dev);
 */
void quaternion_update(zf_device_imu &imu_dev);

/*
函数名称：四元数转欧拉角
功能说明：将当前四元数转换为欧拉角（roll/pitch/yaw 三轴角度）
example：
    quaternion_to_euler();
 */
void quaternion_to_euler(void);

// ==================== 数据获取接口 ====================

/*
函数名称：获取去零偏后的陀螺仪角速度
功能说明：读取原始值、减零偏、转换为 dps
参数说明：imu_dev - 已初始化的 zf_device_imu 对象引用
函数返回：float X轴角速度（dps）
example：
    float gx = get_gyro_x_dps(imu_dev);
 */
float get_gyro_x_dps(zf_device_imu &imu_dev);
float get_gyro_y_dps(zf_device_imu &imu_dev);
float get_gyro_z_dps(zf_device_imu &imu_dev);

/*
函数名称：获取加速度计数据（g单位）
功能说明：读取原始值、转换为 g（1g = 重力加速度）
参数说明：imu_dev - 已初始化的 zf_device_imu 对象引用
函数返回：float 加速度（g）
example：
    float ax = get_acc_x_g(imu_dev);  // 获取X轴加速度，单位g
 */
float get_acc_x_g(zf_device_imu &imu_dev);
float get_acc_y_g(zf_device_imu &imu_dev);
float get_acc_z_g(zf_device_imu &imu_dev);

/*
函数名称：获取加速度计原始数据
功能说明：直接返回 zf_device_imu 的 int16 原始值
参数说明：imu_dev - 已初始化的 zf_device_imu 对象引用
函数返回：int16 加速度原始值
example：
    int16 ax_raw = get_acc_x_raw(imu_dev);
 */
inline int16 get_acc_x_raw(zf_device_imu &imu_dev) { return imu_dev.get_acc_x(); }
inline int16 get_acc_y_raw(zf_device_imu &imu_dev) { return imu_dev.get_acc_y(); }
inline int16 get_acc_z_raw(zf_device_imu &imu_dev) { return imu_dev.get_acc_z(); }

/*
函数名称：获取陀螺仪原始数据
功能说明：直接返回 zf_device_imu 的 int16 原始值
参数说明：imu_dev - 已初始化的 zf_device_imu 对象引用
函数返回：int16 陀螺仪原始值
example：
    int16 gx_raw = get_gyro_x_raw(imu_dev);
 */
inline int16 get_gyro_x_raw(zf_device_imu &imu_dev) { return imu_dev.get_gyro_x(); }
inline int16 get_gyro_y_raw(zf_device_imu &imu_dev) { return imu_dev.get_gyro_y(); }
inline int16 get_gyro_z_raw(zf_device_imu &imu_dev) { return imu_dev.get_gyro_z(); }

/*
函数名称：获取欧拉角
功能说明：直接返回当前欧拉角（度）
example：
    float roll_deg  = get_roll();
    float pitch_deg = get_pitch();
    float yaw_deg   = get_yaw();
 */
float get_roll(void);
float get_pitch(void);
float get_yaw(void);

/*
函数名称：IMU yaw 打印状态机复位
功能说明：把相对 yaw 重新归零，重新进入初始化状态。
*/
void imu_yaw_print_reset(void);

/*
函数名称：IMU yaw 打印状态机
功能说明：按调用周期积分 Z 轴角速度，实时维护相对 yaw，并周期打印 yaw 值。
参数说明：imu_dev - 已初始化的 IMU 对象；dt_s - 本函数调用周期，单位秒。
example：    imu_yaw_print_task(imu_dev, 0.02f);
*/
void imu_yaw_print_task(zf_device_imu &imu_dev, float dt_s);
float imu_get_integrated_yaw(void);

#endif // __IMU660_HPP__
