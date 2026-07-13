#include "zf_common_headfile.hpp"
#include "image.hpp"
#include "myimage.h"
#include "math.h"
#include "init.hpp"
#include <algorithm>

static bool tflite_run(cv::Mat&, int&, float&, long&) { return false; }
static const char *tflite_get_label(int) { return "disabled"; }
int16_t menu_flag = 1;
int ST = 0;

static uint8 Limit_uint8(int low, int value, int high)
{
    if (value < low)
        return (uint8)low;
    if (value > high)
        return (uint8)high;
    return (uint8)value;
}


using namespace cv;
using namespace std;

int ImageScanInterval = 3;
int ImageScanInterval_Cross = 12;

uint8 base_image[UVC_HEIGHT][UVC_WIDTH];
uint8 image[UVC_HEIGHT][UVC_WIDTH];
static uint8 *s_gray_image_ptr = nullptr;
static bool s_jidian_valid = false;
static uint8 s_prev_left_jidian = 2;
static uint8 s_prev_right_jidian = UVC_WIDTH - 2;
uint8 left_jidian, right_jidian;
uint8 left_line_list[UVC_HEIGHT], right_line_list[UVC_HEIGHT], mid_line_list[UVC_HEIGHT];

uint8 l_lost_tip = 0;
uint8 r_lost_tip = 0;
uint8 t_lost_tip = 0;
uint8 b_lost_tip = 0;

uint8 b_lost_num = 0;
uint8 t_lost_num = 0;
uint8 l_lost_num = 0;
uint8 r_lost_num = 0;

enum ImgFlag IF = straightlineS;

////////////////////大津法二值化//////////////////////

/*!
 *  @brief      大津法二值化0.8ms程序
 *  @date:   2018-10
 *  @since      v1.2
 *  *image ：图像地址
 *  width:  图像宽
 *  height：图像高
 *  @author     Z小旋
 */
int otsuThreshold(uint8 *image, uint16 width, uint16 height)
{
#define GrayScale 256
    int pixelCount[GrayScale] = {0}; // 每个灰度值所占像素个数
    int i, j;
    // 舍去图像最上方 Deal_Top 行，不参与阈值计算
    int start_row = (height > Deal_Top) ? Deal_Top : 0;
    int Sumpix = 0; // 有效总像素点（采样后）
    int threshold = 0;
    uint8 *data = image; // 指向像素数据的指针

    // 2x2降采样统计直方图，降低阈值计算耗时
    const int row_step = 2;
    const int col_step = 2;

    // 统计灰度级中每个像素在整幅图像中的个数（降采样）
    for (i = start_row; i < height; i += row_step)
    {
        for (j = 0; j < width; j += col_step)
        {
            pixelCount[(int)data[i * width + j]]++; // 将像素值作为计数数组的下标
            Sumpix++;
            //   pixelCount[(int)image[i][j]]++;    若不用指针用这个
        }
    }

    if (Sumpix <= 0)
    {
        return 0;
    }

    // 全局灰度和
    double sum_all = 0.0;
    for (i = 0; i < GrayScale; i++)
    {
        sum_all += (double)i * (double)pixelCount[i];
    }

    // 使用累计和方式计算类间方差，避免 pow/频繁浮点除法
    double maxVariance = -1.0;
    double sum_b = 0.0;
    int w_b = 0;
    for (int t = 0; t < 256; t++)
    {
        w_b += pixelCount[t];
        if (w_b == 0)
            continue;

        int w_f = Sumpix - w_b;
        if (w_f == 0)
            break;

        sum_b += (double)t * (double)pixelCount[t];

        double m_b = sum_b / (double)w_b;
        double m_f = (sum_all - sum_b) / (double)w_f;
        double diff = m_b - m_f;
        double variance = (double)w_b * (double)w_f * diff * diff; // 类间方差

        if (variance > maxVariance)
        {
            maxVariance = variance;
            threshold = t;
        }
    }
    return threshold;
}

uint8 black_num = 0;

// 二值化稳定参数：像素级滞回
static bool s_prev_binary_valid = false;
static uint8 s_prev_binary[UVC_HEIGHT][UVC_WIDTH];

void set_image_twovalues(int value)
{
    black_num = 0;

    for (uint8 i = 0; i < UVC_HEIGHT; i++)
    {
        if (i <= Deal_Top)
        {
            memset(image[i], 0, UVC_WIDTH);
            continue;
        }

        // 行级阈值参数，避免每个像素重复分支判断
        int hys = 4;
        if (i < Deal_Top + 20)
            hys = 8;
        else if (i < Deal_Top + 40)
            hys = 6;

        const int low_th = value - hys;
        const int high_th = value + hys;
        uint8 *dst_row = image[i];
        const uint8 *src_row = s_gray_image_ptr ? (s_gray_image_ptr + (uint16)i * UVC_WIDTH) : base_image[i];

        if (s_prev_binary_valid)
        {
            const uint8 *prev_row = s_prev_binary[i];
            for (uint8 j = 0; j < UVC_WIDTH; j++)
            {
                uint8 temp_value = src_row[j];

                if (temp_value < low_th)
                    dst_row[j] = 0;
                else if (temp_value > high_th)
                    dst_row[j] = 255;
                else
                    dst_row[j] = prev_row[j];

                // 统计黑像素数量，用于停车线判断
                // if (image[i][j] == 0 && i < 110 && i > 80 && j > 50 && j < 100)
                // {
                //     black_num++;
                // }
            }
        }
        else
        {
            for (uint8 j = 0; j < UVC_WIDTH; j++)
            {
                uint8 temp_value = src_row[j];

                if (temp_value < low_th)
                    dst_row[j] = 0;
                else if (temp_value > high_th)
                    dst_row[j] = 255;
                else
                    dst_row[j] = (temp_value < value ? 0 : 255);

                // 统计黑像素数量，用于停车线判断
                // if (image[i][j] == 0 && i < 110 && i > 80 && j > 50 && j < 100)
                // {
                //     black_num++;
                // }
            }
        }

        image[i][0] = 0;
        image[i][1] = 0;
        image[i][2] = 0;
        image[i][UVC_WIDTH - 2] = 0;
        image[i][UVC_WIDTH - 1] = 0;
    }

    // 保持与丢线信息获取一致的遮罩行为（只需执行一次）
    for (uint8 jj = 0; jj < UVC_WIDTH; jj++)
    {
        image[Deal_Top][jj] = 0;
        image[Deal_Top - 1][jj] = 0;
        image[Deal_Top - 2][jj] = 0;
    }

    // 保存当前帧二值结果，供下一帧滞回参考
    memcpy(s_prev_binary, image, sizeof(s_prev_binary));
    s_prev_binary_valid = true;

    // printf("black_num:%d\n", black_num);
}

//***************************寻找左右基点**************************//
void find_jidian(uint8 index[UVC_HEIGHT][UVC_WIDTH])
{
    // 每帧重新统计底部/顶部丢线计数，避免跨帧累计导致误判
    b_lost_tip = 0;
    t_lost_tip = 0;

    left_jidian = 2;
    right_jidian = UVC_WIDTH - 2;
    const int row = (int)jidian_search_line;

    // 以上一帧两个基点的中点为种子点，向两边找基点
    int mid_seed = MID_W;
    if (s_jidian_valid)
    {
        mid_seed = ((int)s_prev_left_jidian + (int)s_prev_right_jidian) >> 1;
    }
    if (mid_seed < 2)
        mid_seed = 2;
    if (mid_seed > (int)UVC_WIDTH - 2)
        mid_seed = (int)UVC_WIDTH - 2;

    bool left_found = false;
    bool right_found = false;

    // 寻找左边界基点
    for (int j = mid_seed; j >= 2; j--)
    {
        if (index[row][j - 1] == 0 &&
            index[row][j] == 0 &&
            index[row][j + 1] == 255)
        {
            left_jidian = (uint8)j;
            left_found = true;
            break;
        }
    }

    // 寻找右边界基点
    for (int j = mid_seed; j <= (int)UVC_WIDTH - 2; j++)
    {
        if (index[row][j - 1] == 255 &&
            index[row][j] == 0 &&
            index[row][j + 1] == 0)
        {
            right_jidian = (uint8)j;
            right_found = true;
            break;
        }
    }

    // 任一侧没找到时回退到原始全局扫描
    if (!left_found)
    {
        for (int j = 3; j < (int)UVC_WIDTH - 2; j++)
        {
            if (index[row][j - 1] == 0 &&
                index[row][j] == 0 &&
                index[row][j + 1] == 255)
            {
                left_jidian = (uint8)j;
                left_found = true;
                break;
            }
        }
    }
    if (!right_found)
    {
        for (int j = (int)UVC_WIDTH - 3; j > (int)left_jidian; j--)
        {
            if (index[row][j - 1] == 255 &&
                index[row][j] == 0 &&
                index[row][j + 1] == 0)
            {
                right_jidian = (uint8)j;
                right_found = true;
                break;
            }
        }
    }

    // 更新上一帧基点
    s_prev_left_jidian = left_jidian;
    s_prev_right_jidian = right_jidian;
    s_jidian_valid = true;

    // 记录b_lost_tip ,t_lost_tip
    for (uint8 i = Deal_Left; i <= UVC_WIDTH - 2; i++)
    {
        if (image[Lost_Bottom + 1][i] == 255)
        {
            b_lost_tip++;
            image[Lost_Bottom][i] = Lost_line;
        }
        if (image[Lost_Top - 1][i] == 255)
        {
            t_lost_tip++;
            image[Lost_Top][i] = Lost_line;
        }
    }

    // printf("left_jidian:%d,right_jidian:%d\n", left_jidian, right_jidian);
}

//**********************二值化图像去噪*****************************//
void Bin_Image_Filter(void)
{
    uint8 nr; // 行
    uint8 nc; // 列

    // 遍历二值化图像的每个像素点（除去边缘）
    for (nr = 1; nr < UVC_HEIGHT - 1; nr++)
    {
        for (nc = 1; nc < UVC_WIDTH - 1; nc++)
        {
            // 对当前像素点进行条件判断，并根据判断结果进行处理
            if ((image[nr][nc] == 0) && (image[nr - 1][nc] == 255 && image[nr + 1][nc] == 255 && image[nr][nc + 1] == 255 && image[nr][nc - 1] == 255))
            {
                // 如果当前像素为黑色且周围至少有3个白色像素，则将当前像素设为白色
                image[nr][nc] = 255;
            }
            else if ((image[nr][nc] == 255) && (image[nr - 1][nc] == 0 && image[nr + 1][nc] == 0 && image[nr][nc + 1] == 0 && image[nr][nc - 1] == 0))
            {
                // 如果当前像素为白色且周围至少有3个黑色像素，则将当前像素设为黑色
                image[nr][nc] = 0;
            }
        }
    }
}

//**************************搜索左右边线*******************************//
// void image_deal(uint8 index[UVC_HEIGHT][UVC_WIDTH])
// {
//     // 让第一次搜索参照边界基点
//     int left_point = left_jidian;
//     int right_point = right_jidian;

//     for (int i = (int)search_start_line - 1; i > (int)search_end_line; i--)
//     {
//         uint8 left_search_judge = 0;           // 向左搜线标志位
//         uint8 mid_start_left_search_judge = 0; // 从中间向左搜线标志位

//         // 寻找左边界点
//         // 右找10列
//         for (int j = left_point; j < left_point + left_line_right_scarch; j++)
//         {
//             if (index[i][j - 1] == 0 && index[i][j] == 255 && index[i][j + 1] == 255)
//             {
//                 left_point = j;
//                 break;
//             }
//             else if (j == left_point + left_line_right_scarch - 1)
//             {
//                 left_search_judge = 1; // 往右找没找到
//                 break;
//             }
//         }
//         if (left_search_judge == 1)
//         {
//             // 左找5列
//             for (int j = left_point; j > left_point - left_line_left_scarch; j--)
//             {
//                 if (index[i][j - 1] == 0 && index[i][j] == 255 && index[i][j + 1] == 255)
//                 {
//                     left_point = j;
//                     break;
//                 }
//                 else if (j == left_point - left_line_left_scarch + 1)
//                 {
//                     mid_start_left_search_judge = 1; // 往左找没找到
//                     break;
//                 }
//             }
//         }
//         // 往左右找都没有找到,从中点开始往左找
//         if (mid_start_left_search_judge == 1)
//         {
//             for (int j = MID_W; j > 0; j--)
//             {
//                 if (index[i][j - 1] == 0 && index[i][j] == 255 && index[i][j + 1] == 255)
//                 {
//                     left_point = j;
//                     break;
//                 }
//             }
//         }

//         uint8 right_search_judge = 0;           // 向左搜线标志位
//         uint8 mid_start_right_search_judge = 0; // 从中间向左搜线标志位
//         // 寻找右边界点
//         // 左找10列
//         for (int j = right_point; j > right_point - right_line_left_scarch; j--)
//         {
//             if (index[i][j - 1] == 255 && index[i][j] == 255 && index[i][j + 1] == 0)
//             {
//                 right_point = j;
//                 break;
//             }
//             else if (j == right_point - right_line_left_scarch + 1)
//             {
//                 right_search_judge = 1; // 往右找没找到
//                 break;
//             }
//         }
//         if (right_search_judge == 1)
//         {
//             // 右找5列
//             for (int j = right_point; j < right_point + right_line_right_scarch; j++)
//             {
//                 if (index[i][j - 1] == 255 && index[i][j] == 255 && index[i][j + 1] == 0)
//                 {
//                     right_point = j;
//                     break;
//                 }
//                 else if (j == right_point + right_line_right_scarch - 1)
//                 {
//                     mid_start_right_search_judge = 1; // 往左找没找到
//                     break;
//                 }
//             }
//         }
//         // 往左右找都没有找到,从中点开始往右找
//         if (mid_start_right_search_judge == 1)
//         {
//             for (int j = MID_W; j < UVC_WIDTH - 1; j++)
//             {
//                 if (index[i][j - 1] == 255 && index[i][j] == 255 && index[i][j + 1] == 0)
//                 {
//                     right_point = j;
//                     break;
//                 }
//             }
//         }

//         left_line_list[i] = Limit_uint8(1, (uint8)left_point, UVC_WIDTH - 2);
//         right_line_list[i] = Limit_uint8(1, (uint8)right_point, UVC_WIDTH - 2);
//         mid_line_list[i] = Limit_uint8(1, (left_line_list[i] + right_line_list[i]) / 2, UVC_WIDTH - 2);
//         //        printf("%d,%d,%d\r\n",mid_line_list[i],left_line_list[i],right_line_list[i]);
//     }
// }

//*************************************************************八领域******************************************************//

/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
                            uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r            ：最多需要循环的次数
(*image)[image_w]       ：需要进行找点的图像数组，必须是二值图,填入数组名称即可
                       特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic              ：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic              ：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x               ：左边起点横坐标
l_start_y               ：左边起点纵坐标
r_start_x               ：右边起点横坐标
r_start_y               ：右边起点纵坐标
hightest                ：循环结束所得到的最高高度
函数返回：无
修改时间：2022年9月25日
备    注：
example：
    search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */

// 存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = {{0}}; // 左线
uint16 points_r[(uint16)USE_num][2] = {{0}}; // 右线
uint16 dir_r[(uint16)USE_num] = {0};         // 用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = {0};         // 用来存储左边生长方向

// 左边变量
uint8 search_filds_l[8][2] = {{0}};
uint8 index_l = 0;
uint8 temp_l[8][2] = {{0}};
uint16 center_point_l[2] = {0};
uint16 l_data_statics; // 统计左边
// 定义八个邻域
static int8 seeds_l[8][2] = {
    {0, 1},
    {1, 1},
    {1, 0},
    {1, -1},
    {0, -1},
    {-1, -1},
    {-1, 0},
    {-1, 1},
};
//{-1,-1},{0,-1},{+1,-1},
//{-1, 0},       {+1, 0},
//{-1,+1},{0,+1},{+1,+1},
// 这个是逆时针
// 右边变量
uint8 search_filds_r[8][2] = {{0}};
uint8 center_point_r[2] = {0}; // 中心坐标点
uint8 index_r = 0;             // 索引下标
uint8 temp_r[8][2] = {{0}};
uint16 r_data_statics; // 统计右边
// 定义八个邻域

static int8 seeds_r[8][2] = {
    {0, 1},
    {-1, 1},
    {-1, 0},
    {-1, -1},
    {0, -1},
    {1, -1},
    {1, 0},
    {1, 1},
};
//{-1,-1},{0,-1},{+1,-1},
//{-1, 0},       {+1, 0},
//{-1,+1},{0,+1},{+1,+1},
// 这个是顺时针
uint8 hightest = 1; // 爬线相遇点的高度，初始值为1，避免死循环

void search_l_r(uint16 break_flag, uint8 (*image)[UVC_HEIGHT][UVC_WIDTH], uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8 *hightest)
{
    uint8 i = 0;
    bool left_done = false;
    bool right_done = false;
    bool process_left = false;
    bool process_right = false;

    // 每帧重新统计左/右丢线计数，避免跨帧累计导致误判
    l_lost_tip = 0;
    r_lost_tip = 0;

    l_data_statics = 0; // 统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = 0; // 统计找到了多少个点，方便后续把点全部画出来

    // 第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x; // x
    center_point_l[1] = l_start_y; // y
    center_point_r[0] = r_start_x; // x
    center_point_r[1] = r_start_y; // y

    // 开启邻域循环
    while (break_flag--)
    {
        if (left_done && right_done)
            break;

        if (l_data_statics >= USE_num || r_data_statics >= USE_num)
            break;

        // 同步爬线：左右不在同一行时，先让较低一侧追齐
        process_left = false;
        process_right = false;
        if (left_done)
        {
            process_right = !right_done;
        }
        else if (right_done)
        {
            process_left = !left_done;
        }
        else
        {
            if (center_point_l[1] < center_point_r[1])
            {
                process_right = true; // 左边更高，等右边追到同一行
            }
            else if (center_point_l[1] > center_point_r[1])
            {
                process_left = true; // 右边更高，等左边追到同一行
            }
            else
            {
                process_left = true;
                process_right = true; // 同一行则一起往上爬
            }
        }

        if (process_left && !left_done)
        {
            // 中心坐标点填充到已经找到的点内
            points_l[l_data_statics][0] = center_point_l[0]; // x
            points_l[l_data_statics][1] = center_point_l[1]; // y
            l_data_statics++;                                // 索引加一
            index_l = 0;                                     // 先清零，后使用

            if (center_point_l[1] <= Deal_Top)
            {
                left_done = true;
            }
            else
            {
                // 左边判断：在线筛选最优候选，避免额外缓存与二次遍历
                int prev_l_x = points_l[l_data_statics - 1][0];
                int prev_l_y = points_l[l_data_statics - 1][1];
                int prev2_l_x = prev_l_x;
                int prev2_l_y = prev_l_y;
                if (l_data_statics >= 2)
                {
                    prev2_l_x = points_l[l_data_statics - 2][0];
                    prev2_l_y = points_l[l_data_statics - 2][1];
                }

                bool found_l = false;
                int best_l_x = 0, best_l_y = 0;
                for (i = 0; i < 8; i++)
                {
                    const uint8 ni = (uint8)((i + 1) & 7);
                    const uint8 lx = (uint8)(center_point_l[0] + seeds_l[i][0]);
                    const uint8 ly = (uint8)(center_point_l[1] + seeds_l[i][1]);
                    const uint8 lnx = (uint8)(center_point_l[0] + seeds_l[ni][0]);
                    const uint8 lny = (uint8)(center_point_l[1] + seeds_l[ni][1]);

                    if ((*image)[ly][lx] != 0 && (*image)[lny][lnx] == 0)
                    {
                        index_l++;
                        dir_l[l_data_statics - 1] = (i); // 记录生长方向

                        int cx = lnx;
                        int cy = lny;
                        if ((cx == prev_l_x && cy == prev_l_y) || (cx == prev2_l_x && cy == prev2_l_y))
                            continue;

                        if (cy >= Deal_Top && (!found_l || cy < best_l_y))
                        {
                            best_l_x = cx;
                            best_l_y = cy;
                            found_l = true;
                        }
                    }
                }

                // l_lost_tip 记录
                if (center_point_l[0] <= Lost_Left && (*image)[center_point_l[1]][Lost_Left] != Lost_line)
                {
                    l_lost_tip++;
                    (*image)[center_point_l[1]][Lost_Left] = Lost_line;
                }

                // 选择y值最小的候选点（图像最上方的点）作为新边界点，避免回到上一次/上上次点
                if (found_l)
                {
                    center_point_l[0] = (uint8)best_l_x;
                    center_point_l[1] = (uint8)best_l_y;
                }
                else if (center_point_l[1] <= Deal_Top)
                {
                    left_done = true;
                }
            }
        }

        if (process_right && !right_done)
        {
            // 中心坐标点填充到已经找到的点内
            points_r[r_data_statics][0] = center_point_r[0]; // x
            points_r[r_data_statics][1] = center_point_r[1]; // y
            r_data_statics++;
            index_r = 0; // 先清零，后使用

            if (center_point_r[1] <= Deal_Top)
            {
                right_done = true;
            }
            else
            {
                // 右边判断：在线筛选最优候选，避免额外缓存与二次遍历
                int prev_r_x = points_r[r_data_statics - 1][0];
                int prev_r_y = points_r[r_data_statics - 1][1];
                int prev2_r_x = prev_r_x;
                int prev2_r_y = prev_r_y;
                if (r_data_statics >= 2)
                {
                    prev2_r_x = points_r[r_data_statics - 2][0];
                    prev2_r_y = points_r[r_data_statics - 2][1];
                }

                bool found_r = false;
                int best_r_x = 0, best_r_y = 0;
                for (i = 0; i < 8; i++)
                {
                    const uint8 ni = (uint8)((i + 1) & 7);
                    const uint8 rx = (uint8)(center_point_r[0] + seeds_r[i][0]);
                    const uint8 ry = (uint8)(center_point_r[1] + seeds_r[i][1]);
                    const uint8 rnx = (uint8)(center_point_r[0] + seeds_r[ni][0]);
                    const uint8 rny = (uint8)(center_point_r[1] + seeds_r[ni][1]);

                    if ((*image)[ry][rx] != 0 && (*image)[rny][rnx] == 0)
                    {
                        index_r++;                       // 索引加一
                        dir_r[r_data_statics - 1] = (i); // 记录生长方向
                        // printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);

                        int cx = rnx;
                        int cy = rny;
                        if ((cx == prev_r_x && cy == prev_r_y) || (cx == prev2_r_x && cy == prev2_r_y))
                            continue;

                        if (cy >= Deal_Top && (!found_r || cy < best_r_y))
                        {
                            best_r_x = cx;
                            best_r_y = cy;
                            found_r = true;
                        }
                    }
                }

                // r_lost_tip 记录
                if (center_point_r[0] >= Lost_Right && (*image)[center_point_r[1]][Lost_Right] != Lost_line)
                {
                    r_lost_tip++;
                    (*image)[center_point_r[1]][Lost_Right] = Lost_line;
                }

                if (found_r)
                {
                    center_point_r[0] = (uint8)best_r_x;
                    center_point_r[1] = (uint8)best_r_y;
                }
                else if (center_point_r[1] <= Deal_Top)
                {
                    right_done = true;
                }
            }
        }

        if (l_data_statics >= 3 && r_data_statics >= 3)
        {
            if ((points_r[r_data_statics - 1][0] == points_r[r_data_statics - 2][0] &&
                 points_r[r_data_statics - 1][0] == points_r[r_data_statics - 3][0] &&
                 points_r[r_data_statics - 1][1] == points_r[r_data_statics - 2][1] &&
                 points_r[r_data_statics - 1][1] == points_r[r_data_statics - 3][1]) ||
                (points_l[l_data_statics - 1][0] == points_l[l_data_statics - 2][0] &&
                 points_l[l_data_statics - 1][0] == points_l[l_data_statics - 3][0] &&
                 points_l[l_data_statics - 1][1] == points_l[l_data_statics - 2][1] &&
                 points_l[l_data_statics - 1][1] == points_l[l_data_statics - 3][1]))
            {
                // printf("\n连续三点未生长退出\n");
                break;
            }
        }
        if (r_data_statics > 0 && l_data_statics > 0)
        {
            int ri = (int)r_data_statics - 1;
            int li = (int)l_data_statics - 1;
            if (abs((int)points_r[ri][0] - (int)points_l[li][0]) < 2 && abs((int)points_r[ri][1] - (int)points_l[li][1]) < 2)
            {
                // printf("\n左右相遇退出\n");
                *hightest = (points_r[ri][1] + points_l[li][1]) >> 1; // 取出最高点
                // printf("\n在y=%d处退出\n", *hightest);
                break;
            }
            if ((points_r[ri][1] > points_l[li][1]))
            {
                // printf("\n如果左边比右边高了，左边等待右边\n");
                continue; // 如果左边比右边高了，左边等待右边
            }
            if (dir_l[li] == 7 && (points_r[ri][1] > points_l[li][1])) // 左边比右边高且已经向下生长了
            {
                // printf("\n左边开始向下了，等待右边，等待中... \n");
                center_point_l[0] = points_l[li][0]; // x
                center_point_l[1] = points_l[li][1]; // y
                l_data_statics--;
            }
        }
    }
}

/*
函数名称：void get_left(uint16 total_L)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_L ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example： get_left(data_stastics_l );
 */
uint8 l_border[UVC_HEIGHT];    // 左线数组
uint8 r_border[UVC_HEIGHT];    // 右线数组
uint8 center_line[UVC_HEIGHT]; // 中线数组

uint16 dir_right[UVC_HEIGHT] = {0}; // 用来存储右边生长方向
uint16 dir_left[UVC_HEIGHT] = {0};  // 用来存储左边生长方向

void get_left(uint16 total_L)
{
    bool annulus_found = false;
    bool cross_found = false;

    uint16 j = 0;
    l_annulus_point[2][0] = 0;
    l_annulus_point[2][1] = 0;
    l_annulus_point[4][0] = 0; // 十字上拐点
    l_annulus_point[4][1] = 0;
    // 初始化
    std::fill_n(l_border, UVC_HEIGHT, 2);
    std::fill_n(r_border, UVC_HEIGHT, UVC_WIDTH - 2); // 右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
    std::fill_n(dir_left, UVC_HEIGHT, 0);
    uint8 h = UVC_HEIGHT - 2;

    // 左边
    for (j = 0; j < total_L; j++)
    {

        uint16 row = points_l[j][1];
        if (row >= UVC_HEIGHT)
            continue;

        if (annulus_flag != 3 && annulus_flag != 5)
        {
            if (points_l[j][0] >= l_border[row])
            {
                l_border[row] = Limit_uint8(2, points_l[j][0], UVC_WIDTH - 2);
                dir_left[row] = dir_l[j];
                // printf("%d,%d\r\n", row, l_border[row]);
                // printf("%d,%d,%d\r\n", l_border[row], row, dir_left[row]);
            }
        }

        // 找圆环第三拐点
        if (!annulus_found && j > 10 && j + 3 < total_L && row + 5 < UVC_HEIGHT)
        {
            if (dir_l[j + 1] > 1 && dir_l[j + 2] > 1 && points_l[j - 3][1] <= points_l[j][1] && points_l[j + 3][1] < points_l[j][1] && abs(points_l[j][0] - Deal_Left) > 10 && l_border[row + 5] == Deal_Left)
            {
                l_annulus_point[2][0] = points_l[j][0];
                l_annulus_point[2][1] = points_l[j][1];
                annulus_found = true;
                // printf("annulus_l_point3:%d,%d\r\n", l_annulus_point[2][0], l_annulus_point[2][1]);
            }
        }

        if (j > 10 && j + 2 < total_L && row + 5 < UVC_HEIGHT &&
            points_l[j][1] > points_l[j + 2][1] && dir_l[j] > 1 && dir_l[j] <= 3 &&
            dir_l[j + 2] > 1 && !cross_found && l_border[row + 5] == Deal_Left &&
            points_l[j][0] != Deal_Left)
        {
            l_annulus_point[4][0] = points_l[j][0];
            l_annulus_point[4][1] = points_l[j][1];
            cross_found = true;
            // printf("cross_l_point:%d,%d\r\n", l_annulus_point[4][0], l_annulus_point[4][1]);
        }

        if (annulus_flag == 3 || annulus_flag == 5)
        {
            if (points_l[j][1] == h)
            {
                l_border[h] = points_l[j][0];
                dir_left[h] = dir_l[j];
                // printf("%d,%d\r\n", h, l_border[h]);
                // printf("%d,%d,%d\r\n", l_border[h], h, dir_left[h]);

                l_border[h] = Limit_uint8(2, l_border[h], UVC_WIDTH - 2);
            }
            else
                continue; // 每行只取一个点，没到下一行就不记录
            h--;

            if (h == 0)
            {
                break; // 到最后一行退出
            }
        }
    }
}
/*
函数名称：void get_right(uint16 total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example：get_right();
 */
void get_right(uint16 total_R)
{
    bool annulus_found = false;
    bool cross_found = false;

    uint16 j = 0;
    r_annulus_point[2][0] = 0;
    r_annulus_point[2][1] = 0;
    r_annulus_point[4][0] = 0; // 十字上拐点
    r_annulus_point[4][1] = 0;

    std::fill_n(dir_right, UVC_HEIGHT, 0);
    uint8 h = UVC_HEIGHT - 2;

    // 右边
    for (j = 0; j < total_R; j++)
    {

        uint16 row = points_r[j][1];
        if (row >= UVC_HEIGHT)
            continue;

        if (annulus_flag != 3 && annulus_flag != 5)
        {
            if (points_r[j][0] <= r_border[row])
            {
                r_border[row] = Limit_uint8(2, points_r[j][0], UVC_WIDTH - 2);
                dir_right[row] = dir_r[j];

                // printf("%d,%d,%d\r\n", r_border[row], row, dir_right[row]);
            }
        }
        // 找圆环第三拐点
        if (!annulus_found && j > 10 && j + 3 < total_R && row + 5 < UVC_HEIGHT)
        {
            if (dir_r[j + 1] > 1 && dir_r[j + 2] > 1 && points_r[j - 3][1] <= points_r[j][1] && points_r[j + 3][1] < points_r[j][1] && abs(points_r[j][0] - (UVC_WIDTH - 2)) > 10 && r_border[row + 5] == UVC_WIDTH - 2)
            {
                r_annulus_point[2][0] = points_r[j][0];
                r_annulus_point[2][1] = points_r[j][1];
                annulus_found = true;
                // printf("annulus_r_point3:%d,%d\r\n", r_annulus_point[2][0], r_annulus_point[2][1]);
            }
        }

        if (j > 10 && j + 2 < total_R && row + 5 < UVC_HEIGHT &&
            points_r[j][1] > points_r[j + 2][1] && dir_r[j] > 1 && dir_r[j] <= 3 &&
            dir_r[j + 2] > 1 && !cross_found && r_border[row + 5] == UVC_WIDTH - 2 &&
            points_r[j][0] != (UVC_WIDTH - 2))
        {
            r_annulus_point[4][0] = points_r[j][0];
            r_annulus_point[4][1] = points_r[j][1];
            cross_found = true;
            // printf("cross_r_point:%d,%d\r\n", r_annulus_point[4][0], r_annulus_point[4][1]);
        }

        if (annulus_flag == 3 || annulus_flag == 5)
        {
            if (points_r[j][1] == h)
            {
                r_border[h] = points_r[j][0];
                dir_right[h] = dir_r[j];
                // printf("%d,%d\r\n", h, r_border[h]);
                // printf("%d,%d,%d\r\n", r_border[h], h, dir_right[h]);

                r_border[h] = Limit_uint8(2, r_border[h], UVC_WIDTH - 2);
            }
            else
                continue; // 每行只取一个点，没到下一行就不记录
            h--;

            if (h == 0)
            {
                break; // 到最后一行退出
            }
        }
    }
}

void get_center(void)
{
    uint8 h;
    for (h = Deal_Top; h < UVC_HEIGHT - 2; h++)
    {
        center_line[h] = Limit_uint8(Deal_Top, (r_border[h] + l_border[h]) / 2, UVC_WIDTH - 1);
    }
}

/******************************************************************************
 * 函数名称     : void Get_lost_tip(uint8 length)
 * 描述        : 获取四方位边缘特征
 * 进入参数     : void
 * 返回参数     : uint8 length 是正常边界丢失的判断长度
 * 使用实例     ：Get_lost_tip(5);
 * 备注        :lost点没有一定长度,即小于length时，返回判断
 *              底端与左右相连时，仅保留底端
 *              顶端与左右相连时，仅保留顶端
 ******************************************************************************/
uint8 Stop_line = 0;
uint8 dis_Solidline = 0;
uint16 b_center[3] = {0};
uint16 l_center[3] = {0};
uint16 r_center[3] = {0};
uint16 t_center[3] = {0};

// static uint16 last_t_center[2] = {0};
// static uint8 last_t_lost_num = 0;

void Get_lost_tip(uint8 length)
{
    dis_Solidline = 0;
    //    uint8 found = 0;
    //    uint8 found2 = 0;

    // 边缘特征的区域数量初始化
    b_lost_num = 0;
    t_lost_num = 0;
    l_lost_num = 0;
    r_lost_num = 0;
    // 边缘特征的中心点初始化
    memset(b_center, 0, sizeof(b_center));
    memset(l_center, 0, sizeof(l_center));
    memset(r_center, 0, sizeof(r_center));
    memset(t_center, 0, sizeof(t_center));

    // 快速返回：本帧无任何边缘特征
    if (!b_lost_tip && !l_lost_tip && !r_lost_tip && !t_lost_tip)
    {
        return;
    }

    /***********获取底部边缘特征***********/
    if (b_lost_tip)
    {
        for (int i = Deal_Left; i <= UVC_WIDTH - 2; i++) // 从左到右检测紫色丢失线
        {
            if (image[Lost_Bottom][i] == Lost_line) // 检测到紫色丢失线
            {
                if (i <= Deal_Left + length || i >= UVC_WIDTH - 3 - length                           // 在左右临界处检测到紫色丢失线
                    || (i + length <= UVC_WIDTH - 3 && image[Lost_Bottom][i + length] == Lost_line)) // 紫色丢失线有一定长度
                {
                    // 不再计算 b_center，仅保留有效段计数与索引推进
                    for (int k = i; k <= UVC_WIDTH - 2; k++)
                    {
                        if (image[Lost_Bottom][k] == 0) // 判断黑色为紫色丢失线尽头
                        {
                            if (b_lost_num < 3)
                            {
                                b_lost_num++; // 底部区域数量记录
                            }
                            i = k;
                            break;
                        }
                    }
                }
                else // 零碎的杂点，删除处理
                {
                    for (int k = i; k <= i + (int)length && k <= UVC_WIDTH - 2; k++)
                    {
                        if (image[Lost_Bottom][k] == Lost_line)
                        {
                            image[Lost_Bottom][k] = 0;
                            b_lost_tip--;
                        }
                        else if (image[Lost_Bottom][k] == 0)
                        {
                            break;
                        }
                    }
                }
            }
            if (b_lost_num == 2)
            {
                break;
            }
        }
    }

    /***********获取左部边缘特征***********/
    if (l_lost_tip)
    {
        for (int i = UVC_HEIGHT - 4; i >= (int)Deal_Top + 1; i--) // 从下到上检测紫色丢失线
        {
            if (image[i][Lost_Left] == Lost_line) // 检测到紫色丢失线
            {
                if (i + length >= UVC_HEIGHT - 2) // 左侧边缘块过低，考虑底部连续，仅保留底部
                {
                    for (int j = Deal_Left; j <= Deal_Left + (int)length && j <= UVC_WIDTH - 2; j++)
                    {
                        if (image[Lost_Bottom][j] == Lost_line)
                        {
                            image[Lost_Bottom][Lost_Left] = Lost_line;
                            for (; i >= (UVC_HEIGHT - 2) / 2; i--)
                            {
                                if (image[i][Lost_Left] == Lost_line)
                                {
                                    image[i][Lost_Left] = 0;
                                    l_lost_tip--;
                                }
                                else if (image[i][Lost_Left] == 0)
                                {
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
                else // 正常区域，进行记录
                {

                    for (int k = i; i >= (int)Deal_Top; i--)
                    {
                        if (image[i][Lost_Left] == 0)
                        {
                            if (l_lost_num < 3)
                            {
                                l_center[l_lost_num] = (uint16)k - ((uint16)k - ((uint16)i + 1)) / 2;
                                l_lost_num++;
                            }
                            // l_lost_lenth=k-i;
                            break;
                        }
                    }
                }
            }
            if (l_lost_num == 2)
            {
                break;
            }
        }
    }
    /***********获取右部边缘特征***********/
    if (r_lost_tip)
    {
        for (int i = UVC_HEIGHT - 4; i >= (int)Deal_Top + 1; i--) // 从下到上检测紫色丢失线
        {
            if (image[i][Lost_Right] == Lost_line) // 检测到紫色丢失线
            {
                if (i + length >= UVC_HEIGHT - 2) // 右侧边缘块过低，考虑底部连续，仅保留底部
                {
                    for (int j = UVC_WIDTH - 2; j >= UVC_WIDTH - 2 - (int)length && j >= 0; j--)
                    {
                        if (image[Lost_Bottom][j] == Lost_line)
                        {
                            image[Lost_Bottom][Lost_Right] = Lost_line;
                            for (; i >= (UVC_HEIGHT - 2) / 2; i--)
                            {
                                if (image[i][Lost_Right] == Lost_line)
                                {
                                    image[i][Lost_Right] = 0;
                                    r_lost_tip--;
                                }
                                else if (image[i][Lost_Right] == 0)
                                {
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
                else // 正常记录区域
                {
                    int k = i;
                    for (; i >= (int)Deal_Top; i--)
                    {
                        if (image[i][Lost_Right] == 0)
                        {
                            if (r_lost_num < 3)
                            {
                                r_center[r_lost_num] = (uint16)k - ((uint16)k - ((uint16)i + 1)) / 2;
                                r_lost_num++;
                            }
                            break;
                        }
                    }
                }
            }
            if (r_lost_num == 2)
            {
                break;
            }
        }
    }
    /***********获取顶部边缘特征***********/
    if (t_lost_tip)
    {
        // 顶部仅保留与左右联通修正及杂点清理，不再计算 t_center
        for (int i = Deal_Left; i <= UVC_WIDTH - 2; i++)
        {
            if (image[Lost_Top][i] == Lost_line)
            {
                if (i <= Deal_Left + length && l_lost_num) // 区域偏左，且存在左区域，考虑临界区域，保留顶端
                {
                    for (int j = Lost_Top; j <= Lost_Top + (int)length && j < UVC_HEIGHT; j++)
                    {
                        if (image[j][Lost_Left] == Lost_line)
                        {
                            image[Lost_Top][Lost_Left] = Lost_line;
                            for (int k = j; k < (UVC_HEIGHT - 2) / 2; k++)
                            {
                                if (image[k][Lost_Left] == Lost_line && k != Lost_Top)
                                {
                                    l_lost_tip--;
                                    image[k][Lost_Left] = 0;
                                }
                                else if (image[k][Lost_Left] == 0)
                                {
                                    // 不清空左右丢线统计，避免已检测结果被顶部联通逻辑误覆盖
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
                else if (i + length >= UVC_WIDTH - 2 && r_lost_num) // 区域偏右保留顶端
                {
                    for (int k = i; k <= UVC_WIDTH - 2; k++)
                    {
                        for (int j = Lost_Top; j <= Lost_Top + (int)length && j < UVC_HEIGHT; j++)
                        {
                            if (image[j][Lost_Right] == Lost_line)
                            {
                                image[Lost_Top][Lost_Right] = Lost_line;
                                for (int g = j; g < (UVC_HEIGHT - 2) / 2; g++)
                                {
                                    if (image[g][Lost_Right] == Lost_line && g != Lost_Top)
                                    {
                                        r_lost_tip--;
                                        image[g][Lost_Right] = 0;
                                    }
                                    else if (image[g][Lost_Right] == 0)
                                    {
                                        // 不清空左右丢线统计，避免已检测结果被顶部联通逻辑误覆盖
                                        break;
                                    }
                                }
                                break;
                            }
                        }
                    }
                }
            }
        }
        for (int i = Deal_Left; i <= UVC_WIDTH - 2; i++)
        {
            if (image[Lost_Top][i] == Lost_line && ((r_lost_num == 0 || l_lost_num == 0) || (l_lost_num && r_lost_num)))
            {
                for (int k = i; k <= UVC_WIDTH - 2; k++)
                {
                    if (image[Lost_Top][k] == 0)
                    {
                        // 保持原有索引推进行为，避免重复扫描
                        i = k;
                        break;
                    }
                }
            }
            else // 零碎杂点，清除
            {
                for (int k = i; k <= i + (int)length && k <= UVC_WIDTH - 2; k++)
                {
                    if (image[Lost_Top][k] == Lost_line)
                    {
                        image[Lost_Top][k] = 0;
                        t_lost_tip--;
                    }
                    else if (image[Lost_Top][k] == 0)
                    {
                        break;
                    }
                }
            }
        }
        // 不再输出顶部中心
        t_lost_num = 0;
        t_center[0] = 0;
        t_center[1] = 0;
    }
}

void getside_image(uint8 imageInput[UVC_HEIGHT][UVC_WIDTH])
{
    uint8 i = 0, j = 0;
    for (i = 0; i <= UVC_HEIGHT - 1; i++)
    {
        for (j = 0; j <= UVC_WIDTH - 1; j++)
        {
            imageInput[i][j] = 0;
        }
    }

    for (int y = 0; y <= UVC_HEIGHT - 1; y++)
    {
        if (l_border[y] > 0)
            imageInput[y][l_border[y]] = 255;
        if (r_border[y] > 0)
            imageInput[y][r_border[y]] = 255;
        if (r_border[y] > 0)
            imageInput[y][center_line[y]] = 255;
    }
}

// MT9V03X_H 行 MT9V03X_W 列的源图像（src）中裁剪出一个 LCDH 行 LCDW 列的中心区域
void crop_center(const uint8_t src[UVC_HEIGHT][UVC_WIDTH], uint8_t dst[LCDH][LCDW])
{
    const int start_row = UVC_HEIGHT - LCDH;      // 起始行 = 原高度-30（从底部往上保留30行）
    const int start_col = (UVC_WIDTH - LCDW) / 2; // 起始列 = 50

    // 逐行拷贝
    for (int i = 0; i < LCDH; i++)
    {
        const uint8_t *src_ptr = &src[start_row + i][start_col]; // 源起始地址
        uint8_t *dst_ptr = dst[i];                               // 目标起始地址
        memcpy(dst_ptr, src_ptr, LCDW * sizeof(uint8_t));        // 拷贝列
    }
}

// 等比例缩放，保持完整图像
void scale_image(const uint16_t src[UVC_HEIGHT][UVC_WIDTH], uint16_t dst[LCDH][LCDW])
{
    float scale_h = (float)LCDH / UVC_HEIGHT;
    float scale_w = (float)LCDW / UVC_WIDTH;

    for (int i = 0; i < LCDH; i++)
    {
        for (int j = 0; j < LCDW; j++)
        {
            int src_i = (int)(i / scale_h);
            int src_j = (int)(j / scale_w);
            dst[i][j] = src[src_i][src_j];
        }
    }
}

//***********************权重求中线*********************//
// uint8_t mid_weight_list[120] =
//     {
//         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9行: 权重1

//         1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 10-19行: 权重1

//         1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 20-29行: 权重1

//         1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 30-39行: 权重1
//         1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 40-49行: 权重1

//         6, 6, 6, 6, 6, 6, 6, 6, 6, 6, // 50-59行: 权重6

//         7, 8, 9, 10, 11, 12, 13, 14, 15, 16, // 60-69行: 递增

//         17, 18, 19, 20, 20, 20, 20, 19, 18, 17, // 70-79行: 峰值
//         16, 15, 14, 13, 12, 11, 10, 9, 8, 7,    // 80-89行: 递减

//         6, 6, 6, 6, 6, 6, 6, 6, 6, 6, // 90-99行: 权重6

//         1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 100-109行: 权重1

//         1, 1, 1, 1, 1, 1, 1, 1, 1, 1 // 110-119行: 权重1
// };

uint8_t mid_weight_list[UVC_HEIGHT] =
    {
        // 前段低权重 -> 中段抬升 -> 峰值 -> 后段回落
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 10-19
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 20-29
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 30-39

        6, 6, 6, 6, 6, 6, 6, 6, 6, 6, // 40-49

        9, 10, 11, 12, 13, 14, 15, 16, 17, 18, // 50-59

        19, 20, 20, 20, 20, 19, 18, 17, 16, 15, // 60-69

        14, 13, 12, 11, 10, 9, 8, 7, 6, 6, // 70-79

        1, 1, 1, 1, 1, 1, 1, 1, 1, 1 // 80-89
};
static const uint8_t base_mid_weight_list[UVC_HEIGHT] =
    {
        // 前段低权重 -> 中段抬升 -> 峰值 -> 后段回落
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 10-19
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 20-29
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 30-39

        6, 6, 6, 6, 6, 6, 6, 6, 6, 6, // 40-49

        9, 10, 11, 12, 13, 14, 15, 16, 17, 18, // 50-59

        19, 20, 20, 20, 20, 19, 18, 17, 16, 15, // 60-69

        14, 13, 12, 11, 10, 9, 8, 7, 6, 6, // 70-79

        1, 1, 1, 1, 1, 1, 1, 1, 1, 1 // 80-89
};
uint8_t final_mid_line = MID_W; // 最终输出的中线值
uint8_t last_mid_line = MID_W;  // 上次中线值
static uint8 Y_Meet_Array[20] = {2};

uint8 find_mid_line_weight(uint8 target_line[UVC_HEIGHT])
{
    uint8 mid_line_value = MID_W;  // 最终中线输出值
    uint8 mid_line = MID_W;        // 本次中线值
    uint32 weight_midline_sum = 0; // 加权中线累加值
    static uint32 weight_sum = 0;  // 权重累加值（静态缓存）

    if (weight_sum == 0)
    {
        for (int i = UVC_HEIGHT - 1; i > (int)Deal_Top; i--)
            weight_sum += mid_weight_list[i];
        if (weight_sum == 0)
            weight_sum = 1;
    }

    for (int i = UVC_HEIGHT - 1; i > (int)Deal_Top; i--)
    {
        weight_midline_sum += target_line[i] * mid_weight_list[i];
    }
    mid_line = (uint8)(weight_midline_sum / weight_sum);
    mid_line_value = (uint8)(((uint16)last_mid_line * 3 + (uint16)mid_line * 7 + 5) / 10); // 互补滤波
    last_mid_line = mid_line_value;
    return mid_line_value;
}

// 合并中心线生成与加权中线计算，减少一次整段数组遍历
uint8 update_center_and_mid_line_weight(void)
{
    uint8 mid_line_value = MID_W;
    uint8 mid_line = MID_W;
    uint32 weight_midline_sum = 0;
    static uint32 weight_sum = 0;

    if (weight_sum == 0)
    {
        for (int i = UVC_HEIGHT - 1; i > (int)Deal_Top; i--)
            weight_sum += mid_weight_list[i];
        if (weight_sum == 0)
            weight_sum = 1;
    }

    for (int h = (int)Deal_Top; h < (int)UVC_HEIGHT - 2; h++)
    {
        uint8 c = Limit_uint8(Deal_Top, (r_border[h] + l_border[h]) / 2, UVC_WIDTH - 1);
        center_line[h] = c;
        weight_midline_sum += c * mid_weight_list[h];
    }

    mid_line = (uint8)(weight_midline_sum / weight_sum);
    mid_line_value = (uint8)(((uint16)last_mid_line * 3 + (uint16)mid_line * 7 + 5) / 10);
    last_mid_line = mid_line_value;
    return mid_line_value;
}

uint8 Dynamic_Weight_Mid_Line(uint8 start, uint8 *C_Line)
{
    int i = 0;
    int32 Sum = 0;
    int32 Weight_Sum = 0;

    memcpy(mid_weight_list, base_mid_weight_list, sizeof(mid_weight_list));

    // 相遇点滑动均值滤波
    uint8 Y_Meet_Max = Y_Meet_Array[0];
    uint8 Y_Meet_Min = Y_Meet_Array[0];
    uint16 Y_Meet_Sum = 0;
    uint8 Y_Meet_Average = 0;

    // 历史队列右移（从后往前，避免覆盖）
    for (i = 19; i > 0; i--)
    {
        Y_Meet_Array[i] = Y_Meet_Array[i - 1];
    }
    Y_Meet_Array[0] = start;

    for (i = 0; i < 20; i++)
    {
        if (Y_Meet_Array[i] < Y_Meet_Min)
        {
            Y_Meet_Min = Y_Meet_Array[i];
        }
        if (Y_Meet_Array[i] > Y_Meet_Max)
        {
            Y_Meet_Max = Y_Meet_Array[i];
        }
        Y_Meet_Sum += (uint16)Y_Meet_Array[i];
    }

    Y_Meet_Sum = Y_Meet_Sum - (uint16)Y_Meet_Min - (uint16)Y_Meet_Max;
    Y_Meet_Average = (uint8)(Y_Meet_Sum / 18);

    // 按分辨率缩放原始 60 高下的 2~20 限幅
    int y_meet_max_limit = ((int)UVC_HEIGHT * 20) / 60;
    if (y_meet_max_limit < 2)
        y_meet_max_limit = 2;
    if (y_meet_max_limit > (int)UVC_HEIGHT - 2)
        y_meet_max_limit = (int)UVC_HEIGHT - 2;

    if (Y_Meet_Average > (uint8)y_meet_max_limit)
    {
        Y_Meet_Average = (uint8)y_meet_max_limit;
    }
    else if (Y_Meet_Average < 2)
    {
        Y_Meet_Average = 2;
    }

    float Temp_Float = (float)(Y_Meet_Average - 2); // 当直线时，Y_Meet 约为2
    float temp_max = (float)(y_meet_max_limit - 2);
    if (temp_max < 1.0f)
        temp_max = 1.0f;

    if (Temp_Float > temp_max)
    {
        Temp_Float = temp_max;
    }
    if (Temp_Float < 0.0f)
    {
        Temp_Float = 0.0f;
    }

    // 局部动态权值：峰值更宽、过渡更平滑，适合 80 高图像下稳定求中线
    // 前重后轻：增强近处行影响，弯道响应更快，减少外漂
    //     转弯慢/外漂：提高前 6~8 个数，后段继续减
    // 抖动/过冲：降低前 6~8 个数，峰值拉平、变宽
    // 直线稳但弯慢：保持总和不变，前段 +12，后段 -12
    uint8 Trends_Weight[19] = {36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8, 6, 5, 4, 3};

    // 进一步下移动态权重窗口，强化近处行响应（急弯更快内切）
    const int start_line_min = ((int)UVC_HEIGHT) / 2;
    const int start_line_max = ((int)UVC_HEIGHT * 7) / 8;
    uint8 Start_Line = (uint8)(Temp_Float / temp_max * (float)(start_line_max - start_line_min) + (float)start_line_min);

    if (Start_Line > (uint8)(UVC_HEIGHT - 2))
        Start_Line = (uint8)(UVC_HEIGHT - 2);
    if (Start_Line < 18)
        Start_Line = 18; // 需保证 Start_Line - 18 不越界

    // 将动态权值赋值给固定权值数组
    for (i = 0; i < 19; i++)
    {
        mid_weight_list[(int)Start_Line - i] = Trends_Weight[i];
    }

    // 相遇点以上的中线视为无效点，权重置零
    for (i = (int)Deal_Top + 1; i <= (int)Y_Meet_Average && i < (int)UVC_HEIGHT; i++)
    {
        mid_weight_list[i] = 0;
    }

    int begin_row = std::max((int)Deal_Top + 1, (int)Y_Meet_Average + 1);
    int end_row = (int)UVC_HEIGHT - 2;

    for (i = begin_row; i <= end_row; i++) // 加权，可直接加权给中线之后再算偏差值
    {
        Sum += (int32)mid_weight_list[i] * (int32)C_Line[i];
        Weight_Sum += (int32)mid_weight_list[i];
    }

    if (Weight_Sum <= 0)
    {
        return last_mid_line;
    }

    uint8 mid_line = (uint8)(Sum / Weight_Sum);
    mid_line = Limit_uint8(2, mid_line, UVC_WIDTH - 2);

    uint8 mid_line_value = (uint8)(((uint16)last_mid_line * 3 + (uint16)mid_line * 7 + 5) / 10);
    last_mid_line = mid_line_value;
    return mid_line_value;
}

// 合并中心线生成与动态权重中线计算，减少一次整段数组遍历
uint8 update_center_and_dynamic_mid_line(uint8 start)
{
    int i = 0;
    int32 Sum = 0;
    int32 Weight_Sum = 0;

    memcpy(mid_weight_list, base_mid_weight_list, sizeof(mid_weight_list));

    // 相遇点滑动均值滤波
    uint8 Y_Meet_Max = Y_Meet_Array[0];
    uint8 Y_Meet_Min = Y_Meet_Array[0];
    uint16 Y_Meet_Sum = 0;
    uint8 Y_Meet_Average = 0;

    // 历史队列右移（从后往前，避免覆盖）
    for (i = 19; i > 0; i--)
    {
        Y_Meet_Array[i] = Y_Meet_Array[i - 1];
    }
    Y_Meet_Array[0] = start;

    for (i = 0; i < 20; i++)
    {
        if (Y_Meet_Array[i] < Y_Meet_Min)
        {
            Y_Meet_Min = Y_Meet_Array[i];
        }
        if (Y_Meet_Array[i] > Y_Meet_Max)
        {
            Y_Meet_Max = Y_Meet_Array[i];
        }
        Y_Meet_Sum += (uint16)Y_Meet_Array[i];
    }

    Y_Meet_Sum = Y_Meet_Sum - (uint16)Y_Meet_Min - (uint16)Y_Meet_Max;
    Y_Meet_Average = (uint8)(Y_Meet_Sum / 18);

    // 按分辨率缩放原始 60 高下的 2~20 限幅
    int y_meet_max_limit = ((int)UVC_HEIGHT * 20) / 60;
    if (y_meet_max_limit < 2)
        y_meet_max_limit = 2;
    if (y_meet_max_limit > (int)UVC_HEIGHT - 2)
        y_meet_max_limit = (int)UVC_HEIGHT - 2;

    if (Y_Meet_Average > (uint8)y_meet_max_limit)
    {
        Y_Meet_Average = (uint8)y_meet_max_limit;
    }
    else if (Y_Meet_Average < 2)
    {
        Y_Meet_Average = 2;
    }

    float Temp_Float = (float)(Y_Meet_Average - 2); // 当直线时，Y_Meet 约为2
    float temp_max = (float)(y_meet_max_limit - 2);
    if (temp_max < 1.0f)
        temp_max = 1.0f;

    if (Temp_Float > temp_max)
    {
        Temp_Float = temp_max;
    }
    if (Temp_Float < 0.0f)
    {
        Temp_Float = 0.0f;
    }

    // 局部动态权值
    // uint8 Trends_Weight[19] = {36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8, 6, 5, 4, 3};

    uint8 Trends_Weight[19] = {45, 42, 38, 34, 30, 26, 22, 18, 14, 12, 10, 8, 6, 4, 3, 2, 1, 1, 1};

    // 进一步下移动态权重窗口，强化近处行响应（急弯更快内切）
    const int start_line_min = ((int)UVC_HEIGHT) / 2;
    const int start_line_max = ((int)UVC_HEIGHT * 7) / 8;
    uint8 Start_Line = (uint8)(Temp_Float / temp_max * (float)(start_line_max - start_line_min) + (float)start_line_min);

    if (Start_Line > (uint8)(UVC_HEIGHT - 2))
        Start_Line = (uint8)(UVC_HEIGHT - 2);
    if (Start_Line < 18)
        Start_Line = 18; // 需保证 Start_Line - 18 不越界

    // 将动态权值赋值给固定权值数组
    for (i = 0; i < 19; i++)
    {
        mid_weight_list[(int)Start_Line - i] = Trends_Weight[i];
    }

    // 相遇点以上的中线视为无效点，权重置零
    for (i = (int)Deal_Top + 1; i <= (int)Y_Meet_Average && i < (int)UVC_HEIGHT; i++)
    {
        mid_weight_list[i] = 0;
    }

    int begin_row = std::max((int)Deal_Top + 1, (int)Y_Meet_Average + 1);
    int end_row = (int)UVC_HEIGHT - 2;

    for (i = (int)Deal_Top; i < (int)UVC_HEIGHT - 2; i++)
    {
        uint8 c = Limit_uint8(Deal_Top, (r_border[i] + l_border[i]) / 2, UVC_WIDTH - 1);
        center_line[i] = c;
        if (i >= begin_row && i <= end_row)
        {
            Sum += (int32)mid_weight_list[i] * (int32)c;
            Weight_Sum += (int32)mid_weight_list[i];
        }
    }

    if (Weight_Sum <= 0)
    {
        return last_mid_line;
    }

    uint8 mid_line = (uint8)(Sum / Weight_Sum);
    mid_line = Limit_uint8(2, mid_line, UVC_WIDTH - 2);

    uint8 mid_line_value = (uint8)(((uint16)last_mid_line * 3 + (uint16)mid_line * 7 + 5) / 10);
    last_mid_line = mid_line_value;
    return mid_line_value;
}

/*************************************元素*********************************** */

/**
 * @brief 最小二乘法
 * @param uint8 begin				输入起点
 * @param uint8 end					输入终点
 * @param uint8 *border				输入需要计算斜率的边界首地址
 *  @see CTest		Slope_Calculate(start, end, border);//斜率
 */

float Slope_Calculate(uint8 begin, uint8 end, uint8 *border)
{
    float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
    int16 i = 0;
    float result = 0;
    static float resultlast;

    for (i = begin; i > end; i--)
    {
        xsum += i;
        ysum += border[i];
        xysum += i * (border[i]);
        x2sum += i * i;
    }
    if ((begin - end) * x2sum - xsum * xsum) // 判断除数是否为零
    {
        result = ((begin - end) * xysum - xsum * ysum) / ((begin - end) * x2sum - xsum * xsum);
        resultlast = result;
    }
    else
    {
        result = resultlast;
    }
    return result;
}

/**
 * @brief 计算斜率截距
 * @param uint8 start				输入起点
 * @param uint8 end					输入终点
 * @param uint8 *border				输入需要计算斜率的边界
 * @param float *slope_rate			输入斜率地址
 * @param float *intercept			输入截距地址
 *  @see CTest		calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
 * @return 返回说明
 *     -<em>false</em> fail
 *     -<em>true</em> succeed
 */
void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept)
{
    uint16 i, num = 0;
    uint16 xsum = 0, ysum = 0;
    float y_average, x_average;

    num = 0;
    xsum = 0;
    ysum = 0;
    y_average = 0;
    x_average = 0;
    for (i = start; i > end; i--)
    {
        xsum += i;
        ysum += border[i];
        num++;
    }

    // 计算各个平均数
    if (num)
    {
        x_average = (float)(xsum / num);
        y_average = (float)(ysum / num);
    }

    /*计算斜率*/
    *slope_rate = Slope_Calculate(start, end, border);  // 斜率
    *intercept = y_average - (*slope_rate) * x_average; // 截距
}

// 判断是否为直线：根据最大绝对误差和平均绝对误差判断边界是否近似为直线
// 1为直线，0为非直线
bool is_border_straight(const uint8 border[UVC_HEIGHT], uint8 start, uint8 end)
{
    if (start > (uint8)(UVC_HEIGHT - 2))
        start = (uint8)(UVC_HEIGHT - 2);
    if (end > (uint8)(UVC_HEIGHT - 2))
        end = (uint8)(UVC_HEIGHT - 2);
    if (start < (uint8)Deal_Top)
        start = (uint8)Deal_Top;
    if (end < (uint8)Deal_Top)
        end = (uint8)Deal_Top;

    if (start < end)
    {
        uint8 tmp = start;
        start = end;
        end = tmp;
    }

    if (start == end)
        return true;

    int count = 0;
    double sum_x = 0.0, sum_y = 0.0, sum_x2 = 0.0, sum_xy = 0.0;
    for (int i = (int)start; i >= (int)end; i--)
    {
        double x = (double)i;
        double y = (double)border[i];
        sum_x += x;
        sum_y += y;
        sum_x2 += x * x;
        sum_xy += x * y;
        count++;
    }

    if (count < 2)
        return true;

    double denom = (double)count * sum_x2 - sum_x * sum_x;
    if (denom == 0.0)
        return true;

    double slope = ((double)count * sum_xy - sum_x * sum_y) / denom;
    double intercept = (sum_y - slope * sum_x) / (double)count;

    double abs_err_sum = 0.0;
    double max_abs_err = 0.0;
    for (int i = (int)start; i >= (int)end; i--)
    {
        double x = (double)i;
        double y = (double)border[i];
        double y_fit = slope * x + intercept;
        double err = y - y_fit;
        if (err < 0.0)
            err = -err;
        abs_err_sum += err;
        if (err > max_abs_err)
            max_abs_err = err;
    }

    double mean_abs_err = abs_err_sum / (double)count;
    return (max_abs_err <= (double)STRAIGHT_MAX_ABS_ERR) && (mean_abs_err <= (double)STRAIGHT_MEAN_ERR);
}

/******************************************************************************
 * 函数名称     : void left_fill(uint8 *l_border, uint8 start, uint8 end, uint8 flag)
 * 函数名称     : void right_fill(uint8 *r_border, uint8 start, uint8 end, uint8 flag)
 * 描述        : 两点法补线函数
 * 进入参数     : uint8 *l_border 需要补线的边界首地址
 *              uint8 start 输入起点
 *              uint8 end 输入终点
 *              uint8 flag 1：补两点之间；0：全补
 * 返回参数     : void
 * 备注        :连接（x1,y1)和（x2,y2）
 ******************************************************************************/
// 补左线
void left_fill(uint8 *l_border, uint8 start, uint8 end, uint8 flag)
{
    float slope_l_rate = 0.0f, intercept_l = 0.0f;

    if (start >= UVC_HEIGHT)
        start = UVC_HEIGHT - 1;
    if (end >= UVC_HEIGHT)
        end = UVC_HEIGHT - 1;

    if (start < end)
    {
        uint8 tmp = start;
        start = end;
        end = tmp;
    }

    if (start != end)
    {
        slope_l_rate = ((float)l_border[start] - (float)l_border[end]) / ((float)start - (float)end);
        intercept_l = (float)l_border[start] - slope_l_rate * (float)start;
    }
    else
    {
        slope_l_rate = 0.0f;
        intercept_l = (float)l_border[start];
    }

    if (flag == 1)
    {
        for (int i = (int)start; i >= (int)end; i--)
        {
            l_border[i] = slope_l_rate * (float)i + intercept_l; // y = kx+b
            l_border[i] = Limit_uint8(0, l_border[i], UVC_WIDTH - 1);
        }
    }
    else
    {
        for (int i = (int)start; i >= (int)Deal_Top; i--)
        {
            l_border[i] = slope_l_rate * (float)i + intercept_l; // y = kx+b
            l_border[i] = Limit_uint8(2, l_border[i], UVC_WIDTH - 2);
        }
    }
}

// 补右线
void right_fill(uint8 *r_border, uint8 start, uint8 end, uint8 flag)
{
    float slope_r_rate = 0.0f, intercept_r = 0.0f;

    if (start >= UVC_HEIGHT)
        start = UVC_HEIGHT - 1;
    if (end >= UVC_HEIGHT)
        end = UVC_HEIGHT - 1;

    if (start < end)
    {
        uint8 tmp = start;
        start = end;
        end = tmp;
    }

    if (start != end)
    {
        slope_r_rate = ((float)r_border[start] - (float)r_border[end]) / ((float)start - (float)end);
        intercept_r = (float)r_border[start] - slope_r_rate * (float)start;
    }
    else
    {
        slope_r_rate = 0.0f;
        intercept_r = (float)r_border[start];
    }

    if (flag == 1)
    {
        for (int i = (int)start; i >= (int)end; i--)
        {
            r_border[i] = slope_r_rate * (float)i + intercept_r; // y = kx+b
            r_border[i] = Limit_uint8(2, r_border[i], UVC_WIDTH - 2);
        }
    }
    else
    {
        for (int i = (int)start; i >= (int)Deal_Top; i--)
        {
            r_border[i] = slope_r_rate * (float)i + intercept_r; // y = kx+b
            r_border[i] = Limit_uint8(2, r_border[i], UVC_WIDTH - 2);
        }
    }
}

// 十字补左线
void cross_left_fill(uint8 *l_border, uint8 start, uint8 end, uint8 l_cross_left)
{
    float slope_l_rate = 0.0f, intercept_l = 0.0f;

    if (start >= UVC_HEIGHT)
        start = UVC_HEIGHT - 1;
    if (end >= UVC_HEIGHT)
        end = UVC_HEIGHT - 1;
    if (l_cross_left >= UVC_HEIGHT)
        l_cross_left = UVC_HEIGHT - 1;

    if (start < end)
    {
        uint8 tmp = start;
        start = end;
        end = tmp;
    }

    if (start != end)
    {
        slope_l_rate = ((float)l_border[start] - (float)l_border[end]) / ((float)start - (float)end);
        intercept_l = (float)l_border[start] - slope_l_rate * (float)start;
    }
    else
    {
        slope_l_rate = 0.0f;
        intercept_l = (float)l_border[start];
    }

    for (int i = (int)UVC_HEIGHT - 2; i >= (int)l_cross_left; i--)
    {
        l_border[i] = slope_l_rate * (float)i + intercept_l; // y = kx+b
        l_border[i] = Limit_uint8(2, l_border[i], UVC_WIDTH - 2);
    }
}

// 十字补右线
void cross_right_fill(uint8 *r_border, uint8 start, uint8 end, uint8 r_cross_right)
{
    float slope_r_rate = 0.0f, intercept_r = 0.0f;

    if (start >= UVC_HEIGHT)
        start = UVC_HEIGHT - 1;
    if (end >= UVC_HEIGHT)
        end = UVC_HEIGHT - 1;
    if (r_cross_right >= UVC_HEIGHT)
        r_cross_right = UVC_HEIGHT - 1;

    if (start < end)
    {
        uint8 tmp = start;
        start = end;
        end = tmp;
    }

    if (start != end)
    {
        slope_r_rate = ((float)r_border[start] - (float)r_border[end]) / ((float)start - (float)end);
        intercept_r = (float)r_border[start] - slope_r_rate * (float)start;
    }
    else
    {
        slope_r_rate = 0.0f;
        intercept_r = (float)r_border[start];
    }

    for (int i = (int)UVC_HEIGHT - 2; i >= (int)r_cross_right; i--)
    {
        r_border[i] = slope_r_rate * (float)i + intercept_r; // y = kx+b
        r_border[i] = Limit_uint8(2, r_border[i], UVC_WIDTH - 2);
    }
}

// 十字判断
uint8 cross_flag = 0;
uint8 l_cross_point[2] = {0, 0}; // 左拐点
uint8 r_cross_point[2] = {0, 0}; // 右拐点

// 环岛判断
uint8 annulus_flag = 0;
uint8 l_annulus_point[5][2] = {0, 0};                           // 圆环左拐点
uint8 r_annulus_point[5][2] = {0, 0};                           // 圆环右拐点
uint8 cn = 0, cn_cross = 0, cn_l_annulus = 0, cn_r_annulus = 0; // 圆环判断计数
static uint16 left_annulus_inner_frames = 0;
static uint8 left_ring_entry_x = 0;
static uint8 left_ring_entry_y = 0;

bool if_l_annulus = false; // 左圆环前标志
bool if_r_annulus = false; // 右圆环前标志
uint8 if_l_annulus_cn = 0; // 维持状态数
uint8 if_r_annulus_cn = 0;

uint8 l_use[5][2] = {0};
uint8 r_use[5][2] = {0};

uint8 l_cross_use[2] = {0};
uint8 r_cross_use[2] = {0};

bool have_yuansu = false; // 已经跑过元素判断的标志，用于判断停车

// 上次环岛拐点位置，用于补线时判断是否更新拐点位置，避免偶尔一两帧误判导致拐点位置忽上忽下
static uint8 last_l_annulus_point[5][2] = {0};
static uint8 last_r_annulus_point[5][2] = {0};
static void resolve_annulus_points(const uint8 src[5][2], uint8 last[5][2], uint8 dst[5][2])
{
    for (int i = 0; i < 5; i++)
    {
        if (src[i][0] != 0 || src[i][1] != 0)
        {
            dst[i][0] = src[i][0];
            dst[i][1] = src[i][1];
            last[i][0] = src[i][0];
            last[i][1] = src[i][1];
        }
        else
        {
            dst[i][0] = last[i][0];
            dst[i][1] = last[i][1];
        }
    }
}

static uint8 last_l_cross_point[2] = {0};
static uint8 last_r_cross_point[2] = {0};
static void resolve_cross_points(const uint8 src[2], uint8 last[2], uint8 dst[2])
{

    if (src[0] != 0 || src[1] != 0)
    {
        dst[0] = src[0];
        dst[1] = src[1];
        last[0] = src[0];
        last[1] = src[1];
    }
    else
    {
        dst[0] = last[0];
        dst[1] = last[1];
    }
}

void yuansu_judge(void)
{
    bool annulus_found_l1 = false, annulus_found_l2 = false, annulus_found_l4 = false;
    bool annulus_found_r1 = false, annulus_found_r2 = false, annulus_found_r4 = false;
    l_cross_point[0] = 0;
    l_cross_point[1] = 0;
    r_cross_point[0] = 0;
    r_cross_point[1] = 0;
    l_annulus_point[0][0] = 0;
    l_annulus_point[0][1] = 0;
    l_annulus_point[1][0] = 0;
    l_annulus_point[1][1] = 0;
    l_annulus_point[3][0] = 0;
    l_annulus_point[3][1] = 0;
    r_annulus_point[0][0] = 0;
    r_annulus_point[0][1] = 0;
    r_annulus_point[1][0] = 0;
    r_annulus_point[1][1] = 0;
    r_annulus_point[3][0] = 0;
    r_annulus_point[3][1] = 0;

    /****************************************找关键拐点******************************************** */
    for (int i = UVC_HEIGHT - 12; i > Deal_Top + 5; i--)
    {
        // 左
        if (l_border[i] >= l_border[i + 2] && dir_left[i] > 3 && dir_left[i - 1] >= 3 && l_center[0] != 0 && l_center[0] < i && l_border[i - 2] == Deal_Left && l_border[i] != Deal_Left && !annulus_found_l1)
        {

            l_annulus_point[0][0] = l_border[i];
            l_annulus_point[0][1] = i;
            // printf("annulus_l_point1:%d,%d\r\n", l_annulus_point[0][0], l_annulus_point[0][1]);

            l_cross_point[0] = l_border[i]; // x
            l_cross_point[1] = i;           // y
            annulus_found_l1 = true;
        }
        // 第二个拐点
        else if (dir_left[i + 2] > 1 && dir_left[i + 1] <= 3 && dir_left[i] >= 3 && dir_left[i - 1] >= 3 && dir_left[i - 2] >= 3 && abs(l_border[i] - l_border[i - 1]) < 9 && l_border[i] > l_border[i - 2] && l_border[i] >= l_border[i + 2] && l_border[i + 10] != Deal_Left && l_border[i - 5] != Deal_Left && !annulus_found_l2)
        {
            l_annulus_point[1][0] = l_border[i];
            l_annulus_point[1][1] = i;
            annulus_found_l2 = true;
            // printf("annulus_l_point2:%d,%d\r\n", l_annulus_point[1][0], l_annulus_point[1][1]);
        }

        // 找第四个拐点 ：圆环状态5拐点
        if (l_border[i] >= l_border[i + 2] && l_border[i] > l_border[i - 2] && (dir_left[i - 2] >= 3 || dir_left[i - 1] > 3) && !annulus_found_l4)
        {
            l_annulus_point[3][0] = l_border[i];
            l_annulus_point[3][1] = i;
            annulus_found_l4 = true;
        }

        // 右
        if (r_border[i] <= r_border[i + 2] && dir_right[i] > 3 && dir_right[i - 1] >= 3 && r_center[0] != 0 && r_center[0] < i && r_border[i - 2] == UVC_WIDTH - 2 && r_border[i] != UVC_WIDTH - 2 && !annulus_found_r1)
        {
            r_annulus_point[0][0] = r_border[i];
            r_annulus_point[0][1] = i;
            // printf("annulus_r_point1:%d,%d\r\n", r_annulus_point[0][0], r_annulus_point[0][1]);

            r_cross_point[0] = r_border[i]; // x
            r_cross_point[1] = i;           // y
            annulus_found_r1 = true;
        }
        else if (dir_right[i + 2] > 1 && dir_right[i + 1] <= 3 && dir_right[i] >= 3 && dir_right[i - 1] >= 3 && dir_right[i - 2] >= 3 && abs(r_border[i] - r_border[i - 1]) < 9 && r_border[i] < r_border[i - 2] && r_border[i] <= r_border[i + 2] && r_border[i + 10] != UVC_WIDTH - 2 && r_border[i - 5] != UVC_WIDTH - 2 && !annulus_found_r2)
        {
            r_annulus_point[1][0] = r_border[i];
            r_annulus_point[1][1] = i;
            annulus_found_r2 = true;
            // printf("annulus_r_point2:%d,%d\r\n", r_annulus_point[1][0], r_annulus_point[1][1]);
        }

        // 找第四个拐点 ：圆环状态5拐点
        if (r_border[i] <= r_border[i + 2] && r_border[i] < r_border[i - 2] && (dir_right[i - 2] > 3 || dir_right[i - 1] > 3) && !annulus_found_r4)
        {
            r_annulus_point[3][0] = r_border[i];
            r_annulus_point[3][1] = i;
            annulus_found_r4 = true;
        }
    }

    // 第二个点在第三个点上面，舍去第二个点
    if (l_annulus_point[1][1] != 0 && l_annulus_point[2][1] != 0 && l_annulus_point[1][1] <= l_annulus_point[2][1])
    {
        l_annulus_point[1][1] = 0;
        l_annulus_point[1][0] = 0;
        annulus_found_l2 = false;
    }
    if (r_annulus_point[1][1] != 0 && r_annulus_point[2][1] != 0 && r_annulus_point[1][1] <= r_annulus_point[2][1])
    {
        r_annulus_point[1][1] = 0;
        r_annulus_point[1][0] = 0;
        annulus_found_r2 = false;
    }

    // 找到第二个点但未找到第一个点，将第一个点定为最下方
    if (!annulus_found_l1 && annulus_found_l2)
    {
        l_annulus_point[0][1] = UVC_HEIGHT - 2;
        l_annulus_point[0][0] = Deal_Left;
        // printf("annulus_l_point1:%d,%d\r\n", l_annulus_point[0][0], l_annulus_point[0][1]);
    }
    if (!annulus_found_r1 && annulus_found_r2)
    {
        r_annulus_point[0][1] = UVC_HEIGHT - 2;
        r_annulus_point[0][0] = UVC_WIDTH - 2;
        // printf("annulus_r_point1:%d,%d\r\n", r_annulus_point[0][0], r_annulus_point[0][1]);
    }

    // 第一个拐点在第二个上面，将第一个拐点定为最下方
    if (l_annulus_point[0][1] != 0 && l_annulus_point[1][1] != 0 && l_annulus_point[0][1] <= l_annulus_point[1][1])
    {
        l_annulus_point[0][1] = UVC_HEIGHT - 2;
        l_annulus_point[0][0] = Deal_Left;
        // printf("annulus_l_point1:%d,%d\r\n", l_annulus_point[0][0], l_annulus_point[0][1]);
    }
    if (r_annulus_point[0][1] != 0 && r_annulus_point[1][1] != 0 && r_annulus_point[0][1] <= r_annulus_point[1][1])
    {
        r_annulus_point[0][1] = UVC_HEIGHT - 2;
        r_annulus_point[0][0] = UVC_WIDTH - 2;
        // printf("annulus_r_point1:%d,%d\r\n", r_annulus_point[0][0], r_annulus_point[0][1]);
    }

    // printf("cross_l_point:%d,%d\r\n", l_cross_point[0], l_cross_point[1]);
    // printf("cross_r_point:%d,%d\r\n", r_cross_point[0], r_cross_point[1]);
    // printf("annulus_l_point1:%d,%d\r\n", l_annulus_point[0][0], l_annulus_point[0][1]);
    // printf("annulus_l_point2:%d,%d\r\n", l_annulus_point[1][0], l_annulus_point[1][1]);
    // printf("annulus_l_point3:%d,%d\r\n", l_annulus_point[2][0], l_annulus_point[2][1]);
    // printf("annulus_l_point4:%d,%d\r\n", l_annulus_point[3][0], l_annulus_point[3][1]);
    // printf("annulus_l_point5:%d,%d\r\n", l_annulus_point[4][0], l_annulus_point[4][1]);
    // printf("annulus_r_point1:%d,%d\r\n", r_annulus_point[0][0], r_annulus_point[0][1]);
    // printf("annulus_r_point2:%d,%d\r\n", r_annulus_point[1][0], r_annulus_point[1][1]);
    // printf("annulus_r_point3:%d,%d\r\n", r_annulus_point[2][0], r_annulus_point[2][1]);
    // printf("annulus_r_point4:%d,%d\r\n", r_annulus_point[3][0], r_annulus_point[3][1]);
    // printf("annulus_r_point5:%d,%d\r\n", r_annulus_point[4][0], r_annulus_point[4][1]);

    // printf("l_lost_num:%d, l_center:%d,%d\r\n", l_lost_num, l_center[0], l_center[1]);
    // printf("r_lost_num:%d, r_center:%d,%d\r\n", r_lost_num, r_center[0], r_center[1]);

    // printf("hightest:%d\n", hightest);

    resolve_annulus_points(l_annulus_point, last_l_annulus_point, l_use);
    resolve_annulus_points(r_annulus_point, last_r_annulus_point, r_use);
    resolve_cross_points(l_cross_point, last_l_cross_point, l_cross_use);
    resolve_cross_points(r_cross_point, last_r_cross_point, r_cross_use);

    /****************************************找关键拐点******************************************** */

    /*******************************元素判断*************************************************** */

    if (IF == straightlineS && l_lost_num != 0 && l_annulus_point[0][1] > 9 && l_annulus_point[0][0] > 9 && l_annulus_point[1][1] == 0 && r_annulus_point[1][1] == 0 && r_annulus_point[0][1] == 0 && l_annulus_point[0][1] > l_center[0] && r_annulus_point[4][1] == 0 && r_annulus_point[3][1] == 0)
    {
        bool is_straight = is_border_straight(r_border, UVC_HEIGHT - 2, hightest);
        // printf("is_straight:%d\n", is_straight);
        if (is_straight)
        {
            // printf("annulus_r_fill!\n");
            left_fill(l_border, UVC_HEIGHT - 2, l_use[0][1], 0);

            if_l_annulus = true;
            if_l_annulus_cn = 0;
            annulus_biansu = true; // 减速
        }
    }
    else if (IF == straightlineS && r_lost_num != 0 && r_annulus_point[0][1] > 9 && r_annulus_point[0][0] < UVC_WIDTH - 9 && r_annulus_point[1][1] == 0 && l_annulus_point[1][1] == 0 && l_annulus_point[0][1] == 0 && r_annulus_point[0][1] > r_center[0] && l_annulus_point[4][1] == 0 && l_annulus_point[3][1] == 0)
    {

        bool is_straight = is_border_straight(l_border, UVC_HEIGHT - 2, hightest);
        if (is_straight)
        {
            // printf("annulus_l_fill!\n");
            right_fill(r_border, UVC_HEIGHT - 2, r_use[0][1], 0);

            if_r_annulus = true;
            if_r_annulus_cn = 0;
            annulus_biansu = true; // 减速
        }
    }

    if (if_l_annulus)
    {
        if_l_annulus_cn++; // 标志维持三帧
        if (if_l_annulus_cn > 8)
        {
            if_l_annulus = false;
            if_r_annulus_cn = 0;
            if (IF != annulus_l && IF != annulus_r)
                annulus_biansu = false;
        }
    }
    else if (if_r_annulus)
    {
        if_r_annulus_cn++; // 标志维持三帧
        if (if_r_annulus_cn > 8)
        {
            if_r_annulus = false;
            if_r_annulus_cn = 0;
            if (IF != annulus_r && IF != annulus_l)
                annulus_biansu = false;
        }
    }

    // 左右环岛判断
    if (l_annulus_point[1][1] > 3 && r_annulus_point[1][1] == 0 && (!r_lost_num || (r_center[0] != 0 && r_center[0] < l_annulus_point[1][1])) && l_lost_num != 0 && IF == straightlineS)
    {

        if (if_l_annulus) // 连续多帧满足条件才判断为环岛，避免偶尔一两帧误判
        {
            // printf("annulus_l!\n");
            IF = annulus_l;
            annulus_flag = 1; // 初见左环岛
            // printf("annulus_flag:%d\n", annulus_flag);
        }
    }
    else if (r_annulus_point[1][1] > 3 && l_annulus_point[1][1] == 0 && (!l_lost_num || (l_center[0] != 0 && l_center[0] < r_annulus_point[1][1])) && r_lost_num != 0 && IF == straightlineS)
    {

        if (if_r_annulus) // 连续多帧满足条件才判断为环岛，避免偶尔一两帧误判
        {
            // printf("annulus_r!\n");
            IF = annulus_r;
            annulus_flag = 1; // 初见右环岛
            // printf("annulus_flag:%d\n", annulus_flag);
        }
    }

    // 十字：普通弯道也可能短暂出现 2~3 个假拐点，所以这里收紧为“四点完整 + 双侧明显丢线”。
    bool left_cross_real = l_cross_point[1] > 18;
    bool right_cross_real = r_cross_point[1] > 18;
    bool left_cross_top = l_annulus_point[4][1] > 25;
    bool right_cross_top = r_annulus_point[4][1] > 25;

    if (!left_cross_real)
        l_cross_point[1] = 0;
    if (!right_cross_real)
        r_cross_point[1] = 0;
    if (!left_cross_top)
        l_annulus_point[4][1] = 0;
    if (!right_cross_top)
        r_annulus_point[4][1] = 0;

    bool cross_shape_ok = left_cross_real && right_cross_real && left_cross_top && right_cross_top;
    bool cross_lost_ok = l_lost_num >= 2 && r_lost_num >= 2;

    if (cross_shape_ok && cross_lost_ok && IF == straightlineS)
    {
        cn_cross++;
        if (cn_cross > 3) // 连续多帧满足条件才判断为十字，避免普通弯道误判
        {
            // printf("crossroads!\n");
            IF = crossroads;
            cn_cross = 0;
            cross_flag = 1;
            have_yuansu = true; // 已经跑过元素判断，识别到停车线后可以停车
        }
    }
    else
    {
        cn_cross = 0;
    }

    // 十字状态判断
    if (IF == crossroads)
    {
        if ((l_cross_point[1] == 0 || (l_cross_point[1] != 0 && l_annulus_point[4][1] > l_cross_point[1])) && (r_cross_point[1] == 0 || (r_cross_point[1] != 0 && r_annulus_point[4][1] > r_cross_point[1])) && cross_flag == 1)
        {
            cn++;
            if (cn > 2)
            {
                cross_flag = 2; // 十字完全进入
                cn = 0;
                // printf("cross_flag:%d\n", cross_flag);
            }
        }

        // if (l_annulus_point[4][1] == 0 && r_annulus_point[4][1] == 0)
        else if (l_annulus_point[4][1] == 0 && r_annulus_point[4][1] == 0 && cross_flag == 2)
        {
            cn++;
            if (cn > 2)
            {
                IF = straightlineS;
                cn = 0;
                // printf("cross_flag:%d\n", cross_flag);
            }
        }
        else
            cn = 0;
    }

    /******************************************  岛内状态判断***********************************/
    if (IF == annulus_l)
    {
        // 岛内状态判断（按右环岛逻辑对称改写）
        if (l_annulus_point[1][1] != 0 && (!r_lost_num || r_center[0] < l_annulus_point[1][1]) && annulus_flag == 1)
        {
            annulus_flag = 2; // 初入圆环
            // printf("annulus_flag:%d\n", annulus_flag);
        }
        else if ((l_annulus_point[1][1] == 0 || l_annulus_point[1][1] > 50) && l_annulus_point[2][1] > 10 && annulus_flag == 2)
        {
            cn++;
            if (cn > 2)
            {
                annulus_flag = 3; // 入环
                // printf("annulus_l_point3:%d,%d\r\n", l_annulus_point[2][0], l_annulus_point[2][1]);
                // printf("annulus_flag:%d\n", annulus_flag);
                cn = 0;
            }
        }
        else if (l_annulus_point[2][1] == 0 && annulus_flag == 3)
        {
            cn++;
            if (cn > 4) // 连续多帧丢失3点才进下一个状态，避免偶尔一两帧误判
            {
                annulus_flag = 4; // 环内正常循迹
                left_annulus_inner_frames = 0;
                printf("[RING][L] flag=4 环内正常循迹\r\n");
                cn = 0;
            }
        }
        else if (annulus_flag == 4)
        {
            left_annulus_inner_frames++;
            bool exit_point_valid = (r_annulus_point[3][1] > (UVC_HEIGHT * 4) / 5 &&
                                     r_annulus_point[3][0] > (UVC_WIDTH * 3) / 4);
            if (exit_point_valid && left_annulus_inner_frames > 140)
            {
                cn++;
                if (cn > 4)
                {
                    annulus_flag = 5; // 出环
                    printf("[RING][L] flag=5 出环 point4=(%d,%d) inner=%d\r\n",
                           r_annulus_point[3][0], r_annulus_point[3][1], left_annulus_inner_frames);
                    cn = 0;
                }
            }
            else
            {
                cn = 0;
            }
        }
        else if (l_annulus_point[2][1] != 0 && (r_annulus_point[3][1] == 0 || r_annulus_point[3][1] < l_annulus_point[2][1]) && annulus_flag == 5) // 第三个点又找到了，认为出圆环
        {
            cn++;
            if (cn > 2)
            {
                annulus_flag = 6; // 出圆环
                // printf("annulus_flag:%d\n", annulus_flag);
                cn = 0;
            }
        }
        else if ((l_annulus_point[2][1] == 0 || r_annulus_point[2][1] < UVC_HEIGHT / 2) && l_annulus_point[1][1] == 0 && l_border[UVC_HEIGHT - 5] != Deal_Left && annulus_flag == 6) // 第二、三个点丢失
        {
            cn++;
            if (cn > 2)
            {
                annulus_flag = 7; // 出圆环，正常循迹，清空标志位
                // printf("annulus_flag:%d\n", annulus_flag);
                cn = 0;
            }
        }
        else
            cn = 0;
    }
    else if (IF == annulus_r)
    {
        // 岛内状态判断
        if (r_annulus_point[1][1] != 0 && (!l_lost_num || l_center[0] < r_annulus_point[1][1]) && annulus_flag == 1)
        {
            annulus_flag = 2; // 初入园环
            // printf("annulus_flag:%d\n", annulus_flag);
        }
        else if ((r_annulus_point[1][1] == 0 || r_annulus_point[1][1] > 50) && r_annulus_point[2][1] > 10 && annulus_flag == 2)
        {
            cn++;
            if (cn > 2) // 连续多帧满足条件才判断为入环，避免偶尔一两帧误判
            {
                annulus_flag = 3; // 入环
                // printf("annulus_r_point3:%d,%d\r\n", r_annulus_point[2][0], r_annulus_point[2][1]);
                // printf("annulus_flag:%d\n", annulus_flag);
                cn = 0;
            }
        }
        else if (r_annulus_point[2][1] == 0 && annulus_flag == 3)
        {
            cn++;
            if (cn > 4) // 连续多帧丢失3点才进下一个状态，避免偶尔一两帧误判
            {
                annulus_flag = 4; // 环内正常循迹
                // printf("annulus_flag:%d\n", annulus_flag);
                cn = 0;
            }
        }
        else if (l_annulus_point[3][1] != 0 && l_annulus_point[3][0] < 150 && annulus_flag == 4)
        {
            cn++;
            if (cn > 2)
            {
                annulus_flag = 5; // 出环
                // printf("annulus_flag:%d\n", annulus_flag);
                cn = 0;
                // printf("l_annulus_point0:%d,%d\r\n", l_annulus_point[0][0], l_annulus_point[0][1]);
            }
        }
        else if (r_annulus_point[2][1] != 0 && (l_annulus_point[3][1] == 0 || l_annulus_point[3][1] < r_annulus_point[2][1]) && annulus_flag == 5) // 第二个点又找到了，认为出圆环
        {
            cn++;
            if (cn > 2)
            {
                annulus_flag = 6; // 出圆环
                // printf("annulus_flag:%d\n", annulus_flag);
                cn = 0;
            }
        }
        else if ((r_annulus_point[2][1] == 0 || r_annulus_point[2][1] < UVC_HEIGHT / 2) && r_annulus_point[1][1] == 0 && r_border[UVC_HEIGHT - 5] != UVC_WIDTH - 2 && annulus_flag == 6) // 第二、三个点丢失
        {
            cn++;
            if (cn > 2)
            {
                annulus_flag = 7; // 出圆环，正常循迹，清空标志位
                // printf("annulus_flag:%d\n", annulus_flag);
                cn = 0;
            }
        }
        else
            cn = 0;
    }

    // printf("IF=%d\n", IF);
    // printf("annulus_flag:%d\n", annulus_flag);
    // printf("cross_flag:%d\n", cross_flag);

    /*******************************判断*************************************************** */
}

// 环岛处理
extern float left_T, left_base;
extern float right_T, right_base;
bool annulus_biansu = false; // 0正常，1减速

// 十字处理
void run_crossroads(void)
{
    if (IF == crossroads)
    {
        if (cross_flag == 1)
        {
            l_border[l_use[4][1]] = l_use[4][0];
            r_border[r_use[4][1]] = r_use[4][0];

            // l_border[l_use[0][1]] = l_use[0][0];
            // r_border[r_use[0][1]] = r_use[0][0];

            if (l_cross_use[1] != 0 && l_use[4][1] != 0 && l_cross_use[1] > l_use[4][1])
            {
                left_fill(l_border, l_cross_use[1], l_use[4][1], 1);
            }
            else if (l_cross_use[1] == 0 && l_use[4][1] != 0)
            {
                left_fill(l_border, UVC_HEIGHT - 2, l_use[4][1], 1);
            }
            else if ((l_use[4][1] == 0 && l_cross_use[1] != 0) || (l_use[4][1] != 0 && l_cross_use[1] != 0 && l_cross_use[1] < l_use[4][1]))
            {
                // left_fill(l_border, UVC_HEIGHT - 2, l_use[0][1], 0);
                cross_left_fill(l_border, l_cross_use[1] + 10, l_cross_use[1] + 5, Deal_Top);
            }
            // else if (l_use[0][1] == 0 && l_use[4][1] == 0)
            // {
            //     cross_left_fill(l_border, UVC_HEIGHT - 5, UVC_HEIGHT - 15, Deal_Top);
            // }

            if (r_cross_use[1] != 0 && r_use[4][1] != 0 && r_cross_use[1] > r_use[4][1])
            {
                right_fill(r_border, r_cross_use[1], r_use[4][1], 1);
            }
            else if (r_cross_use[1] == 0 && r_use[4][1] != 0)
            {
                right_fill(r_border, UVC_HEIGHT - 2, r_use[4][1], 1);
            }
            else if ((r_use[4][1] == 0 && r_cross_use[1] != 0) || (r_use[4][1] != 0 && r_cross_use[1] != 0 && r_cross_use[1] < r_use[4][1]))
            {
                // right_fill(r_border, UVC_HEIGHT - 2, r_use[0][1], 0);
                cross_right_fill(r_border, r_cross_use[1] + 10, r_cross_use[1] + 5, Deal_Top);
                // printf("cross_right_fill!\n");
            }
            // else if (l_use[0][1] == 0 && l_use[4][1] == 0)
            // {
            //     cross_right_fill(r_border, UVC_HEIGHT - 5, UVC_HEIGHT - 15, Deal_Top);
            // }
        }
        else if (cross_flag == 2)
        {

            l_border[l_use[4][1]] = l_use[4][0];
            r_border[r_use[4][1]] = r_use[4][0];

            // left_fill(l_border, UVC_HEIGHT - 2, l_use[4][1], 1);
            uint8 l_cross_start = (l_use[4][1] > 5) ? (uint8)(l_use[4][1] - 5) : 0;
            uint8 l_cross_end = (l_use[4][1] > 10) ? (uint8)(l_use[4][1] - 10) : 0;
            uint8 r_cross_start = (r_use[4][1] > 5) ? (uint8)(r_use[4][1] - 5) : 0;
            uint8 r_cross_end = (r_use[4][1] > 10) ? (uint8)(r_use[4][1] - 10) : 0;
            cross_left_fill(l_border, l_cross_start, l_cross_end, l_use[4][1]);
            // right_fill(r_border, UVC_HEIGHT - 2, r_use[4][1], 1);
            cross_right_fill(r_border, r_cross_start, r_cross_end, r_use[4][1]);
        }
    }
}

void run_annulus(void)
{
    // 左环岛
    if (IF == annulus_l)
    {
        // 环岛补线
        if (annulus_flag == 1 || annulus_flag == 2) // 初见环岛或入环岛
        {
            // annulus_biansu = true; // 减速

            left_fill(l_border, l_use[0][1], l_use[1][1], 0);
            // printf("buxian1!\n");
        }
        else if (annulus_flag == 3) // 入环岛
        {
            r_border[UVC_HEIGHT - 2] = UVC_WIDTH - 2; // 从右下角开始补右线
            r_border[l_use[2][1]] = l_use[2][0];
            if (left_ring_entry_y == 0 && l_use[2][1] > 0)
            {
                left_ring_entry_x = l_use[2][0];
                left_ring_entry_y = l_use[2][1];
            }
            if (left_ring_entry_y > 0)
            {
                r_border[left_ring_entry_y] = left_ring_entry_x;
            }
            static int left_ring_fill_print_div = 0;
            if (++left_ring_fill_print_div >= 10)
            {
                left_ring_fill_print_div = 0;
                printf("[RINGFILL][L][3] right_fill start=(%d,%d) end=(%d,%d)\r\n",
                       UVC_WIDTH - 2, UVC_HEIGHT - 2,
                       left_ring_entry_y > 0 ? left_ring_entry_x : l_use[2][0],
                       left_ring_entry_y > 0 ? left_ring_entry_y : l_use[2][1]);
            }
            right_fill(r_border, UVC_HEIGHT - 2,
                       left_ring_entry_y > 0 ? left_ring_entry_y : l_use[2][1], 0);
        }
        else if (annulus_flag == 4) // 环岛内正常循迹
        {
            if (left_annulus_inner_frames < 110 && left_ring_entry_y > 0)
            {
                r_border[UVC_HEIGHT - 2] = UVC_WIDTH - 2;
                r_border[left_ring_entry_y] = left_ring_entry_x;
                right_fill(r_border, UVC_HEIGHT - 2, left_ring_entry_y, 0);
            }
        }
        else if (annulus_flag == 5) // 出环
        {
            if (r_use[3][1] != 0)
            {
                r_border[2] = Deal_Left;
                right_fill(r_border, r_use[3][1], 2, 0); // 把右线补到左上角
            }
            else
            {
                r_border[2] = Deal_Left;
                r_border[UVC_HEIGHT - 2] = UVC_WIDTH - 2;
                right_fill(r_border, UVC_HEIGHT - 2, 2, 0); // 把右线补到左上角
            }
        }
        else if (annulus_flag == 6) // 出圆环
        {
            if (l_use[2][1] != 0)
            {
                l_border[l_use[2][1]] = l_use[2][0];
                // cross_left_fill(l_border, l_use[2][1] - 5, l_use[2][1] - 8, l_use[2][1]);
                left_fill(l_border, UVC_HEIGHT - 2, l_use[2][1], 1);
            }
        }
        else if (annulus_flag == 7) // 出圆环，正常循迹，清空标志位
        {
            annulus_biansu = false;

            IF = straightlineS;
            annulus_flag = 0;
            left_annulus_inner_frames = 0;
            left_ring_entry_x = 0;
            left_ring_entry_y = 0;
            // printf("annulus_end!\n");
        }
    }

    // 右环岛
    if (IF == annulus_r)
    {

        // 环岛补线
        if (annulus_flag == 1 || annulus_flag == 2) // 初见环岛或入环岛
        {
            // annulus_biansu = true; // 减速

            right_fill(r_border, r_use[0][1], r_use[1][1], 0);
        }
        else if (annulus_flag == 3) // 入环岛
        {

            l_border[r_use[2][1]] = r_use[2][0];
            left_fill(l_border, UVC_HEIGHT - 2, r_use[2][1], 0);
        }
        else if (annulus_flag == 4) // 环岛内正常循迹
        {
        }
        else if (annulus_flag == 5) // 出环
        {
            if (l_use[3][1] != 0)
            {
                l_border[2] = UVC_WIDTH - 2;
                left_fill(l_border, l_use[3][1], 2, 0); // 把左线补到右上角
            }
            else
            {
                l_border[2] = UVC_WIDTH - 2;
                l_border[UVC_HEIGHT - 2] = Deal_Left;
                left_fill(l_border, UVC_HEIGHT - 2, 2, 0); // 把左线补到右上角
            }
        }
        else if (annulus_flag == 6) // 出圆环
        {
            if (r_use[2][1] != 0)
            {
                r_border[r_use[2][1]] = r_use[2][0];
                // cross_right_fill(r_border, r_use[2][1] - 5, r_use[2][1] - 8, r_use[2][1]);
                right_fill(r_border, UVC_HEIGHT - 2, r_use[2][1], 1);
            }
        }
        else if (annulus_flag == 7) // 出圆环，正常循迹，清空标志位
        {
            annulus_biansu = false;

            IF = straightlineS;
            annulus_flag = 0;
            // printf("ann·ulus_end!\n");
        }
    }
}

void run_startline(void)
{
    if (IF == startline)
    {
        cleanup();
    }
}

void yuansu_run(void)
{
    yuansu_judge();
    run_crossroads();
    run_annulus();
    run_startline();
}

uint8 *gray_image;
int bin_threshold;
extern int16_t menu_flag;
extern int ST;
bool is_straight_lr = true;

void track(void)
{
    // auto start_time = std::chrono::high_resolution_clock::now();

    gray_image = uvc_cam.get_gray_image_ptr();
    if (gray_image == nullptr)
        return;
    s_gray_image_ptr = gray_image;

    bin_threshold = otsuThreshold(gray_image, UVC_WIDTH, UVC_HEIGHT);
    set_image_twovalues(bin_threshold);
    // Bin_Image_Filter();
    find_jidian(image); // 找基点
    search_l_r((uint16)USE_num, &image, left_jidian, UVC_HEIGHT - 2, right_jidian, UVC_HEIGHT - 2, &hightest);
    // printf("\n在y=%d处退出\n", hightest);

    Get_lost_tip(5);

    get_left(l_data_statics);
    get_right(r_data_statics);

    if (menu_flag != 0)
    {
        yuansu_run();
        // printf("annulus_flag:%d\n", annulus_flag);
    }

    // final_mid_line = update_center_and_mid_line_weight();

    //   get_center();
    // final_mid_line = Dynamic_Weight_Mid_Line(hightest, center_line);

    final_mid_line = update_center_and_dynamic_mid_line(hightest);
    if (IF == annulus_l && annulus_flag == 4)
    {
        int biased_mid = (int)final_mid_line + 18;
        if (biased_mid > UVC_WIDTH - 2)
            biased_mid = UVC_WIDTH - 2;
        final_mid_line = (uint8)biased_mid;
    }

    is_straight_lr = is_border_straight(r_border, UVC_HEIGHT - 2, hightest) && is_border_straight(l_border, UVC_HEIGHT - 2, hightest);
        if(is_straight_lr)
        {
            ST = 1;
        }
        else
        {
            ST = 0;
        }
    static int last_debug_if = -1;
    static int last_debug_annulus_flag = -1;
    static int ring_debug_div = 0;
    bool ring_debug_active = (IF == annulus_l || IF == annulus_r || annulus_flag != 0);
    bool ring_debug_changed = ring_debug_active &&
                              (last_debug_if != (int)IF || last_debug_annulus_flag != (int)annulus_flag);
    if (ring_debug_changed || (ring_debug_active && ++ring_debug_div >= 10))
    {
        ring_debug_div = 0;
        printf("[RINGDBG] IF=%d flag=%d h=%d mid=%d lost L=%d R=%d lC=%d,%d rC=%d,%d\r\n",
               IF, annulus_flag, hightest, final_mid_line, l_lost_num, r_lost_num,
               l_center[0], l_center[1], r_center[0], r_center[1]);
        printf("[RINGDBG] Lp1=(%d,%d) Lp2=(%d,%d) Lp3=(%d,%d) Lp4=(%d,%d) Lp5=(%d,%d)\r\n",
               l_annulus_point[0][0], l_annulus_point[0][1],
               l_annulus_point[1][0], l_annulus_point[1][1],
               l_annulus_point[2][0], l_annulus_point[2][1],
               l_annulus_point[3][0], l_annulus_point[3][1],
               l_annulus_point[4][0], l_annulus_point[4][1]);
        printf("[RINGDBG] Rp1=(%d,%d) Rp2=(%d,%d) Rp3=(%d,%d) Rp4=(%d,%d) Rp5=(%d,%d)\r\n",
               r_annulus_point[0][0], r_annulus_point[0][1],
               r_annulus_point[1][0], r_annulus_point[1][1],
               r_annulus_point[2][0], r_annulus_point[2][1],
               r_annulus_point[3][0], r_annulus_point[3][1],
               r_annulus_point[4][0], r_annulus_point[4][1]);
        last_debug_if = (int)IF;
        last_debug_annulus_flag = (int)annulus_flag;
    }

    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    // printf("元素判断耗时: %ld us\n", duration);
}

// 视觉识别已经迁移到 CarVision，这里保留旧接口但不再做红色识别、裁切或推理。
int red_x = 0, red_y = 0;
int black_block_count = 0;
bool infer_valid = false;
bool roi_valid = false;
static int last_infer_class_idx = -1;
static int same_class_streak = 0;
static int stable_infer_class_idx = -1;

bool red_find(cv::Mat)
{
    red_x = 0;
    red_y = 0;
    return false;
}

void black_find(cv::Mat)
{
    black_block_count = 0;
}

int infer_judge(cv::Mat)
{
    infer_valid = false;
    roi_valid = false;
    last_infer_class_idx = -1;
    same_class_streak = 0;
    stable_infer_class_idx = -1;
    return -1;
}

uint8 tflite_flag = 0;
int frame_count = 0; // 帧计数器
int16_t frame_time;  // 连续帧数阈值，超过则认为元素消失

void run_infer(int)
{
    tflite_flag = 0;
    frame_count = 0;
    infer_valid = false;
    roi_valid = false;
    last_infer_class_idx = -1;
    same_class_streak = 0;
    stable_infer_class_idx = -1;
}


// ================= Nova compatibility layer =================
uint8_t Image_Use[LCDH][LCDW];
uint8_t Pixle[LCDH][LCDW];
ImageStatustypedef ImageStatus;
SystemDatatypdef SystemData;
ImageDealDatatypedef ImageDeal[LCDH];
ImageFlagtypedef ImageFlag;
ImageStatustypedef ImageData;
ROIRegionTypedef LargestWhiteRegion;
bool Cross_State_Print = false;

static int clamp_nova_int(int value, int low, int high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

static void sync_myimage_to_nova_status(void)
{
    ImageStatus.Left_Line = 0;
    ImageStatus.Right_Line = 0;
    ImageStatus.WhiteLine = 0;

    for (int y = 0; y < LCDH; y++)
    {
        int src_y = y * 2;
        if (src_y >= UVC_HEIGHT)
            src_y = UVC_HEIGHT - 1;

        int left = clamp_nova_int((int)l_border[src_y] / 2, 0, LCDW - 1);
        int right = clamp_nova_int((int)r_border[src_y] / 2, 0, LCDW - 1);
        int center = clamp_nova_int((int)center_line[src_y] / 2, 0, LCDW - 1);
        if (right <= left)
        {
            left = 0;
            right = LCDW - 1;
            center = LCDW / 2;
        }

        ImageDeal[y].LeftBorder = left;
        ImageDeal[y].RightBorder = right;
        ImageDeal[y].Center = center;
        ImageDeal[y].Wide = right - left;
        ImageDeal[y].LeftBoundary = left;
        ImageDeal[y].RightBoundary = right;
        ImageDeal[y].LeftBoundary_First = left;
        ImageDeal[y].RightBoundary_First = right;
        ImageDeal[y].IsLeftFind = (l_border[src_y] <= Deal_Left + 1) ? 'W' : 'T';
        ImageDeal[y].IsRightFind = (r_border[src_y] >= UVC_WIDTH - 3) ? 'W' : 'T';
        ImageDeal[y].Black_Point = 0;

        if (ImageDeal[y].IsLeftFind == 'W')
            ImageStatus.Left_Line++;
        if (ImageDeal[y].IsRightFind == 'W')
            ImageStatus.Right_Line++;
        if (ImageDeal[y].IsLeftFind == 'W' && ImageDeal[y].IsRightFind == 'W')
            ImageStatus.WhiteLine++;

        for (int x = 0; x < LCDW; x++)
        {
            int src_x = x * 2;
            int sum_gray = 0;
            int sum_bin = 0;
            for (int dy = 0; dy < 2; dy++)
            {
                for (int dx = 0; dx < 2; dx++)
                {
                    int yy = clamp_nova_int(src_y + dy, 0, UVC_HEIGHT - 1);
                    int xx = clamp_nova_int(src_x + dx, 0, UVC_WIDTH - 1);
                    sum_gray += base_image[yy][xx];
                    sum_bin += image[yy][xx] > 0 ? 1 : 0;
                }
            }
            Image_Use[y][x] = (uint8_t)(sum_gray / 4);
            Pixle[y][x] = (sum_bin >= 2) ? 1 : 0;
        }
    }

    ImageStatus.Det_True = clamp_nova_int((int)final_mid_line / 2, 0, LCDW - 1);
    ImageStatus.TowPoint_True = ImageStatus.TowPoint;
    ImageStatus.OFFLine = clamp_nova_int((int)hightest / 2, 0, LCDH - 1);

    ImageFlag.image_element_rings_flag = 0;
    ImageFlag.image_element_rings = 0;
    ImageFlag.ring_big_small = 0;
    if (IF == annulus_l)
    {
        ImageStatus.Road_type = LeftCirque;
        ImageFlag.image_element_rings = 1;
        ImageFlag.image_element_rings_flag = annulus_flag;
        ImageFlag.ring_big_small = 1;
    }
    else if (IF == annulus_r)
    {
        ImageStatus.Road_type = RightCirque;
        ImageFlag.image_element_rings = 2;
        ImageFlag.image_element_rings_flag = annulus_flag;
        ImageFlag.ring_big_small = 1;
    }
    else if (IF == crossroads)
    {
        ImageStatus.Road_type = Cross_ture;
    }
    else
    {
        ImageStatus.Road_type = is_straight_lr ? Straight : Normol;
    }
}

void ImageProcess(void)
{
    track();
    sync_myimage_to_nova_status();
}

float my_abs(float x)
{
    return x < 0.0f ? -x : x;
}

void DrawLine() {}
void Element_Test(void) {}
void Element_Handle(void) {}
void FindLargestWhiteRegion(ROIRegionTypedef *roi)
{
    if (roi == nullptr)
        return;
    roi->area = 0;
    roi->left = 0;
    roi->right = 0;
    roi->top = 0;
    roi->bottom = 0;
    roi->center_x = 0;
    roi->center_y = 0;
}
