#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_
#include "zf_common_typedef.hpp"
#include <opencv2/opencv.hpp>

#define jidian_search_line UVC_HEIGHT - 1 // 基点搜索行
#define search_start_line UVC_HEIGHT      // 搜线起始行
#define search_end_line 30                // 搜线停止行
#define left_line_right_scarch 10         // 搜索左边线时向右搜索几列
#define left_line_left_scarch 5           // 搜索左边线时向左搜索几列
#define right_line_left_scarch 10         // 搜索右边线时向左搜索几列
#define right_line_right_scarch 5         // 搜索右边线时向右搜索几列
#define MID_W 75                          // 赛道中值
#define USE_num 300                       // 定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

#define Control_line 254 // 红色
#define Left_line 253    // 绿色
#define Right_line 252   // 蓝色
#define Lost_line 251    // 紫色
#define Judge_line 250   // 青色
#define Make_up 249      // 粉色
#define LandR 248        // 橙色
#define Deal_Left 2      // 最左
#define Deal_Top 2       // 最上
#define Lost_Left Deal_Left + 2
#define Lost_Right UVC_WIDTH - 4
#define Lost_Bottom UVC_HEIGHT - 3
#define Lost_Top Deal_Top + 2

// 直线判定阈值（像素）
#define STRAIGHT_MAX_ABS_ERR 5
#define STRAIGHT_MEAN_ERR 2

#ifndef LCDH
#define LCDH UVC_HEIGHT / 2 // LCD显示高度
#endif
#ifndef LCDW
#define LCDW UVC_WIDTH / 2  // LCD显示宽度
#endif

enum ImgFlag
{
    straightlineS, // 短直道
    straightlineL, // 长直道
    annulus_l,     // 左环岛
    annulus_r,     // 右环岛
    crossroads,    // 十字路口
    startline,     // 停车线
}; // 图像标志位

extern uint8 *gray_image;

extern uint8 base_image[UVC_HEIGHT][UVC_WIDTH];
extern uint8 image[UVC_HEIGHT][UVC_WIDTH];
extern uint8 left_jidian, right_jidian;
extern uint8 left_line_list[UVC_HEIGHT], right_line_list[UVC_HEIGHT], mid_line_list[UVC_HEIGHT];
extern uint8 final_mid_line; // 最终输出的中线值
extern uint8 last_mid_line;  // 上次中线值
extern int otsuThreshold(uint8 *image, uint16 width, uint16 height);

extern uint8 annulus_flag;

extern uint8 l_border[UVC_HEIGHT];    // 左线数组
extern uint8 r_border[UVC_HEIGHT];    // 右线数组
extern uint8 center_line[UVC_HEIGHT]; // 中线数组

extern uint8 b_lost_num;
extern uint8 t_lost_num;
extern uint8 l_lost_num;
extern uint8 r_lost_num;

extern uint16 dir_right[UVC_HEIGHT]; // 用来存储右边生长方向
extern uint16 dir_left[UVC_HEIGHT];  // 用来存储左边生长方向
extern uint8 l_cross_point[2];       // 十字左拐点
extern uint8 r_cross_point[2];       // 十字右拐点
extern uint8 l_annulus_point[5][2];  // 圆环左拐点
extern uint8 r_annulus_point[5][2];  // 圆环右拐点

extern bool annulus_biansu; // 0正常，1减速

extern bool if_l_annulus; // 左圆环前标志
extern bool if_r_annulus; // 右圆环前标志

extern uint8 startline_cn; // 停车线计数

extern uint8 tflite_flag;

extern uint16 l_center[3];
extern uint16 r_center[3];
extern uint16 t_center[3];

void set_image_twovalues(int value);
void find_jidian(uint8 index[UVC_HEIGHT][UVC_WIDTH]);
void image_deal(uint8 index[UVC_HEIGHT][UVC_WIDTH]);
uint8 find_mid_line_weight(void);
void getside_image(uint8 imageInput[UVC_HEIGHT][UVC_WIDTH]);
void search_l_r(uint16 break_flag, uint8 (*image)[UVC_HEIGHT][UVC_WIDTH], uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8 *hightest);
void get_left(uint16 total_L);
void get_right(uint16 total_R);
void get_center(void);
void Get_lost_tip(uint8 length);
void connect_line_l(int x1, int y1, int x2, int y2);
void connect_line_r(int x1, int y1, int x2, int y2);
void connect_line_c(int x1, int y1, int x2, int y2);
uint8 get_yuansu(void);
void crop_center(const uint8_t src[UVC_HEIGHT][UVC_WIDTH], uint8_t dst[LCDH][LCDW]);
void scale_image(const uint8_t src[UVC_HEIGHT][UVC_WIDTH], uint8_t dst[LCDH][LCDW]);
void Bin_Image_Filter(void);
bool is_border_straight(const uint8 border[UVC_HEIGHT], uint8 start, uint8 end);

void yuansu_run(void);
void track(void);

bool red_find(cv::Mat src_img);
void black_find(cv::Mat hsv);
int infer_judge(cv::Mat src_img);
void run_infer(int infer_class);

#endif
