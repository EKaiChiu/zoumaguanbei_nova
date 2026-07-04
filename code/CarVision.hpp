#ifndef CAR_VISION_HPP
#define CAR_VISION_HPP

// 返回值：
// -1 = 当前没有识别到有效目标
//  0 = 武器
//  1 = 物资
//  2 = 载具

bool vision_init();
int vision_get();
void vision_close();

#endif
