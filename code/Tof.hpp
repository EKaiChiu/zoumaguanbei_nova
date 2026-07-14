#ifndef TOF_HPP
#define TOF_HPP

#include <stdint.h>

// TOF距离接口：封装DL1X库读取，并提供简单滤波后的缓存值。
// 注意：不要在硬实时中断里直接调用 tof_update()，它会读取sysfs文件，可能阻塞。
bool tof_init(void);
bool tof_is_ready(void);
void tof_update(void);

int tof_get_raw_mm(void);
int tof_get_filtered_mm(void);
bool tof_has_fresh_data(uint32_t max_age_ms);
uint32_t tof_get_last_update_ms(void);
bool tof_is_distance_valid(int distance_mm);

#endif // TOF_HPP
