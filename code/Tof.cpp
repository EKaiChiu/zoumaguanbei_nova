#include "Tof.hpp"

#include "zf_device_dl1x.hpp"

#include <stdio.h>
#include <time.h>

static zf_device_dl1x tof_dev;
static bool tof_ready = false;
static int tof_raw_mm = -1;
static int tof_filtered_mm = -1;
static uint32_t tof_last_update_ms = 0;

static uint32_t tof_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

bool tof_is_distance_valid(int distance_mm)
{
    return distance_mm > 0 && distance_mm < 4000 && distance_mm != 8192;
}

bool tof_init(void)
{
    enum dl1x_device_type_enum type = tof_dev.init();
    tof_ready = (type != NO_FIND_DEVICE);
    tof_raw_mm = -1;
    tof_filtered_mm = -1;
    tof_last_update_ms = 0;

    if (tof_ready)
        printf("[TOF] init done, type=%d.\r\n", type);
    else
        printf("[TOF] init failed, disabled.\r\n");

    return tof_ready;
}

bool tof_is_ready(void)
{
    return tof_ready;
}

void tof_update(void)
{
    if (!tof_ready)
        return;

    int distance = tof_dev.get_distance();
    tof_raw_mm = distance;

    if (!tof_is_distance_valid(distance))
        return;

    if (tof_filtered_mm < 0)
        tof_filtered_mm = distance;
    else
        tof_filtered_mm = (tof_filtered_mm * 3 + distance) / 4;

    tof_last_update_ms = tof_now_ms();
}

int tof_get_raw_mm(void)
{
    return tof_raw_mm;
}

int tof_get_filtered_mm(void)
{
    return tof_filtered_mm;
}

uint32_t tof_get_last_update_ms(void)
{
    return tof_last_update_ms;
}

bool tof_has_fresh_data(uint32_t max_age_ms)
{
    if (!tof_ready || tof_filtered_mm < 0 || tof_last_update_ms == 0)
        return false;

    return (uint32_t)(tof_now_ms() - tof_last_update_ms) <= max_age_ms;
}
