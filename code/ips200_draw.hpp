#ifndef ips200_draw_HPP
#define ips200_draw_HPP
#include "zf_common_headfile.hpp"
typedef struct red_detect_t red_detect;  // 前向声明，避免循环包含
#include "red_detect.hpp"
void draw_left_line(void);        // 画左边界线（🔴红色）
void draw_right_line(void);       // 画右边界线（🔴红色）  
void draw_center_line(void);      // 画中线轨迹（🔵蓝色）
void draw_offline(void);          // 画丢线位置标记（🔴红色横线）
void draw_towpoint_up(void);      // 画前瞻点上界（🟢青色横线）
void draw_towpoint_down(void);    // 画前瞻点下界（🟢青色横线）
void ips200_screen_display(void); // ⭐ 主显示函数：整合所有基础绘制（真彩色模式）

// ==================== 高级绘制功能（推荐使用！）====================
// 绘制到screen_buf缓冲区，然后一次性刷新屏幕（消除闪烁）
// 用于RGB565彩色原图 + 标注线的显示模式 ⭐⭐最常用
void draw_boundary_on_screen(uint16 screen_buf[][160]);   // 🔴左右边界线→RGB565缓冲区
void draw_trajectory_on_screen(uint16 screen_buf[][160]); // 🔵中线轨迹→RGB565缓冲区
void draw_offline_line_on_screen(uint16 screen_buf[][160]); // 🔴丢线位置横线→RGB565缓冲区
int towpoint_get_up_row(void);
int towpoint_get_down_row(void);
void towpoint_set_up_row(int row);
void towpoint_set_down_row(int row);
void draw_towpoint_lines_on_screen(uint16 screen_buf[][160]); // 🟢前瞻点范围横线→RGB565缓冲区
void draw_red_target_on_screen(uint16 screen_buf[][160], red_detect *result); //🟢 青色目标标注→RGB565缓冲区
// ==================== 图传标注功能 ====================
// 为逐飞助手的灰度图像添加调试标注（浅灰色边界+中灰色轨迹）
void draw_annotation_on_imagecopy(void);  // 边界+轨迹→灰度image_copy数组



#endif

