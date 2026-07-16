#include "Test.hpp"
#include "imu660.hpp"

#include <stdio.h>

static bool test_yaw_print_enabled = false;
static bool test_yaw_print_last_enabled = false;
static int test_yaw_print_count = 0;

static const int TEST_YAW_PRINT_INTERVAL = 5; // 5 * 20ms = 100ms

void test_init(void)
{
    test_yaw_print_last_enabled = test_yaw_print_enabled;
    test_yaw_print_count = 0;
}

void test_update(zf_device_imu &imu_dev, float dt_s)
{
    (void)imu_dev;
    (void)dt_s;

    if (!test_yaw_print_enabled)
    {
        if (test_yaw_print_last_enabled)
        {
            printf("[TEST] yaw print off\r\n");
        }
        test_yaw_print_last_enabled = false;
        test_yaw_print_count = 0;
        return;
    }

    if (!test_yaw_print_last_enabled)
    {
        imu_yaw_print_reset();
        test_yaw_print_count = 0;
        test_yaw_print_last_enabled = true;
        printf("[TEST] yaw print on\r\n");
    }

    test_yaw_print_count++;
    if (test_yaw_print_count >= TEST_YAW_PRINT_INTERVAL)
    {
        test_yaw_print_count = 0;
        printf("[TEST][YAW] %.1f\r\n", imu_get_integrated_yaw());
    }
}

const char *test_get_param_name(int index)
{
    switch (index)
    {
    case TEST_PARAM_YAW_PRINT:
        return "YawPrn";
    default:
        return "Unknown";
    }
}

float test_get_param_value(int index)
{
    switch (index)
    {
    case TEST_PARAM_YAW_PRINT:
        return test_yaw_print_enabled ? 1.0f : 0.0f;
    default:
        return 0.0f;
    }
}

void test_set_param_value(int index, float value)
{
    switch (index)
    {
    case TEST_PARAM_YAW_PRINT:
        test_yaw_print_enabled = (value >= 0.5f);
        break;
    default:
        break;
    }
}

void test_adjust_param(int index, int direction)
{
    switch (index)
    {
    case TEST_PARAM_YAW_PRINT:
        if (direction > 0)
            test_yaw_print_enabled = true;
        else if (direction < 0)
            test_yaw_print_enabled = false;
        else
            test_yaw_print_enabled = !test_yaw_print_enabled;
        break;
    default:
        break;
    }
}

bool test_get_yaw_print_enabled(void)
{
    return test_yaw_print_enabled;
}

void test_set_yaw_print_enabled(bool enabled)
{
    test_yaw_print_enabled = enabled;
}
