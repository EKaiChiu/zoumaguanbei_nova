#ifndef TEST_HPP
#define TEST_HPP

#include "zf_common_headfile.hpp"
#include "zf_device_imu.hpp"

// Test 调试模块参数。
// 后续新增测试功能时，可以继续往这里加枚举和菜单项。
enum TestParamIndex
{
    TEST_PARAM_YAW_PRINT = 0, // 是否周期打印 yaw：0=关闭，1=打开
    TEST_PARAM_COUNT
};

void test_init(void);
void test_update(zf_device_imu &imu_dev, float dt_s);

const char *test_get_param_name(int index);
float test_get_param_value(int index);
void test_set_param_value(int index, float value);
void test_adjust_param(int index, int direction);

bool test_get_yaw_print_enabled(void);
void test_set_yaw_print_enabled(bool enabled);

#endif
