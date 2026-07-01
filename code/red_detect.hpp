#ifndef __RED_DETECT_HPP__
#define __RED_DETECT_HPP__

#include "zf_common_headfile.hpp"

typedef struct red_detect_t
{   int is_red;          // 是否有红色
    int is_found;        // 是否找到红色目标
    int center_x;        // 红色目标中心 x
    int center_y;        // 红色目标中心 y
    int red_lower_bound; // 红色像素下边界 y 坐标
    int red_upper_bound; // 红色像素上边界 y 坐标
    int red_left_bound;  // 红色像素左边界 x 坐标
    int red_right_bound; // 红色像素右边界 x 坐标
    int red_area;         // 红色像素数量
    int dist_to_bottom;   // 红色块中心到屏幕底部的像素距离
} red_detect;

void red_detect_first(uint16 *rgb_image, red_detect *result);
void color_detect(uint16 *rgb_image);
#endif // __RED_DETECT_HPP__