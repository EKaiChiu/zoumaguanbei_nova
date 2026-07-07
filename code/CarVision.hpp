#ifndef CAR_VISION_HPP
#define CAR_VISION_HPP

#include <stdint.h>

// 返回值：
// -1 = 当前没有识别到有效目标
//  0 = 武器左侧
//  1 = 物资右侧
//  2 = 载具

bool vision_init();
int vision_get();
int vision_get_from_rgb565(const uint16_t *rgb565, int width, int height);
void vision_close();

#endif
