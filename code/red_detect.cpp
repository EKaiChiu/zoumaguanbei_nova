#include "red_detect.hpp"

#define RED_ROW_RATIO 0.10f // 红色像素占扫描宽度的比例超过此值视为红色行,当前为10%

#define RED_CONFIRM_ROWS 3 // 色块确认所需连续红色行数,当前为3行

#define RED_TOP_GAP_MAX 2 // 向上找色块顶部时允许的最大间断行数,当前为2行

#define search_bottom 110 // 扫描起始行
#define search_top 10     // 扫描结束行
/*
函数名称：红色像素判定
功能说明：解码RGB565像素，判断是否为红色
参数说明：
    pixel：RGB565格式的单个像素值
函数返回：1=红色, 0=非红色
修改时间：2026年6月19日
备    注：
    判定条件（绝对值法）：
      1. R > 120              —— 红色通道足够亮
      2. G < 110 && B < 110  —— 绿蓝通道不能太高（排除白/黄）
      3. R > G + 40           —— 红色显著高于绿色
      4. R > B + 40           —— 红色显著高于蓝色
      未实测过红色能否识别
example：
    if (is_red_pixel(rgb_image[y * 160 + x])) { red_num++; }
 */
static int is_red_pixel(uint16 pixel)
{
    int r5 = (pixel >> 11) & 0x1F;
    int g6 = (pixel >> 5) & 0x3F;
    int b5 = pixel & 0x1F;

    int r = r5 * 255 / 31;
    int g = g6 * 255 / 63;
    int b = b5 * 255 / 31;

    if (r > 120 && g < 110 && b < 110 && r > g + 40 && r > b + 40)
        return 1;
    return 0;
}

/*
函数名称：单行红色像素统计
功能说明：统计指定行扫描区间内的红色像素数量
参数说明：
    rgb_image：RGB565格式图像（160×120）
    y：目标行号
    x_start：扫描起始列
    x_end：扫描结束列（不含）
函数返回：该行扫描区间内的红色像素个数
修改时间：2026年6月19日
备    注：
    此函数在find_red_block中被逐行调用（热路径），保持实现简单以利于编译器优化
example：
    int red_cnt = count_red_in_row(rgb_image, 100, 8, 152);
 */
static int count_red_in_row(uint16 *rgb_image, int y, int x_start, int x_end)
{
    int cnt = 0;
    for (int x = x_start; x < x_end; x++)
    {
        if (is_red_pixel(rgb_image[y * 160 + x]))
            cnt++;
    }
    return cnt;
}

/*
函数名称：红色色块定位
功能说明：自下而上逐行扫描，定位红色色块的完整边界（上下左右）
参数说明：
    rgb_image：RGB565格式图像（160×120）
    result：输出结果结构体指针，写入边界和中心坐标
函数返回：1=找到色块, 0=未找到
修改时间：2026年6月19日
备    注：
    算法流程（移植自Abandoned项目red_rect.cpp）：
      [1] 预计算每行红色像素占比（跳过左右各5%边缘避免噪声）
      [2] 自下而上逐行扫描，找到连续RED_CONFIRM_ROWS行满足RED_ROW_RATIO
      [3] 从确认位置向下补全真正的底边
      [4] 向上找顶部，允许RED_TOP_GAP_MAX行间断（应对透视边缘不整齐）
      [5] 在色块中间行，从左/右各向内扫描精确左右边界
example：
    find_red_block(rgb_image, &result);
 */
static int find_red_block(uint16 *rgb_image, red_detect *result)
{
    // 左右搜索边缘
    int margin = UVC_WIDTH * 0.05;
    int scan_x_start = margin;
    int scan_x_end = UVC_WIDTH - margin;
    int scan_width = scan_x_end - scan_x_start;

    // ---- [1] 预计算每行红色像素占比 ----
    float row_ratios[120];
    for (int y = 0; y < UVC_HEIGHT; y++)
    {
        row_ratios[y] = 0;
    }
    for (int y = UVC_HEIGHT - 1; y >= 1; y--)
    {
        row_ratios[y] = (float)count_red_in_row(rgb_image, y, scan_x_start, scan_x_end) / (float)scan_width;
    }

    // ---- [2] 自下而上，找连续 RED_CONFIRM_ROWS 行满足阈值 ----
    int confirmed_bottom = -1; // 确认的色块底部行号
    int red_row_count = 0;     // 连续红色行计数器

    for (int y = search_bottom; y >= search_top; y--)
    {
        if (row_ratios[y] >= RED_ROW_RATIO)
        {
            red_row_count++;
            if (red_row_count >= RED_CONFIRM_ROWS)
            {
                confirmed_bottom = y;
                break;
            }
        }
        else
        {
            red_row_count = 0;
        }
    }

    if (confirmed_bottom < 0)
        return 0;

    // ---- [3] 从确认底部向下补全真正的底边 ----
    int blk_bottom = confirmed_bottom;
    for (int y = confirmed_bottom + 1; y < UVC_HEIGHT - 5; y++)
    {
        if (row_ratios[y] >= RED_ROW_RATIO)
            blk_bottom = y;
        else
            break;
    }

    // ---- [4] 向上找顶部，允许最多 RED_TOP_GAP_MAX 行间断 ----
    int blk_top = confirmed_bottom;
    int gap = 0;
    for (int y = confirmed_bottom - 1; y >= 0; y--)
    {
        if (row_ratios[y] >= RED_ROW_RATIO)
        {
            blk_top = y;
            gap = 0;
        }
        else
        {
            gap++;
            if (gap > RED_TOP_GAP_MAX)
                break;
        }
    }

    // ---- [5] 在色块中间行精确扫描左右边界 ----
    int mid_y = (blk_top + blk_bottom) / 2;
    int lx = -1, rx = -1;

    for (int x = scan_x_start; x < scan_x_end; x++)
    {
        if (is_red_pixel(rgb_image[mid_y * 160 + x]))
        {
            lx = x;
            break;
        }
    }
    for (int x = scan_x_end; x >= scan_x_start; x--)
    {
        if (is_red_pixel(rgb_image[mid_y * 160 + x]))
        {
            rx = x;
            break;
        }
    }

    if (lx < 0 || rx < 0 || rx <= lx)
        return 0;

    // 写入结果
    result->red_upper_bound = blk_top;
    result->red_lower_bound = blk_bottom;
    result->red_left_bound = lx;
    result->red_right_bound = rx;
    result->center_x = (lx + rx) / 2;
    result->center_y = (blk_top + blk_bottom) / 2;
    result->dist_to_bottom = 120 - result->center_y;  // 像素距离
    result->is_found = 1;
    return 1;
}

// =============================================================================
// 公开接口
// =============================================================================

/*
函数名称：红色检测初步
功能说明：全图扫描统计红色像素，若超过阈值则进一步定位色块边界
参数说明：
    rgb_image：输入图像，RGB565格式，分辨率160x120
    result：输出结果结构体指针
函数返回：无
修改时间：2026年6月19日
备    注：
    - is_red:   是否有足够红色像素（面积 > 50）
    - is_found: 是否成功定位到色块（含完整边界坐标）
    - 全图扫描 + 色块定位共两趟遍历，160×120 在嵌入式平台约 2~4ms
example：
    red_detect result;
    red_detect_first(uvc_cam.get_rgb_image_ptr(), &result);
    if (result.is_found) {
        // 使用 result.center_x, result.red_lower_bound 等
    }
 */
void red_detect_first(uint16 *rgb_image, red_detect *result)
{
    // 空指针保护
    if (rgb_image == NULL || result == NULL)
    {
        if (result != NULL)
        {
            result->is_red = 0;
            result->is_found = 0;
        }
        return;
    }

    // ---- 第一趟：全图扫描，统计红色像素总数 ----
    int red_num = 0;
    for (int y = 0; y < 120; y++)
    {
        for (int x = 0; x < 160; x++)
        {
            if (is_red_pixel(rgb_image[y * 160 + x]))
                red_num++;
        }
    }

    result->red_area = red_num;
    result->is_red = (red_num > 50) ? 1 : 0;

    // ---- 第二趟：色块边界定位（仅在红色够多时执行） ----

    result->is_found = 0;
    result->center_x = 0;
    result->center_y = 0;
    result->red_upper_bound = 0;
    result->red_lower_bound = 0;
    result->red_left_bound = 0;
    result->red_right_bound = 0;
    result->dist_to_bottom = -1;

    if (result->is_red)
    {
        find_red_block(rgb_image, result); // 成功则覆盖 is_found=1 + 坐标
    }
}

/*
函数名称：底部中间像素颜色检测
功能说明：读取图像底部中间一个像素的RGB565值，解码并打印RGB数值
参数说明：
    rgb_image：RGB565格式图像（160×120）
函数返回：无
修改时间：2026年6月21日
备    注：
    取底部中间像素 (UVC_WIDTH/2, UVC_HEIGHT-1)，解码为8位RGB后通过串口打印
example：
    color_detect(uvc_cam.get_rgb_image_ptr());
 */
void color_detect(uint16 *rgb_image)
{
    if (rgb_image == NULL)
        return;

    int x = UVC_WIDTH / 2;  // 中间列 = 80
    int y = UVC_HEIGHT - 1; // 最后一行（底部）= 119

    uint16 pixel = rgb_image[y * 160 + x];

    // 解码 RGB565 → 8位 RGB
    int r5 = (pixel >> 11) & 0x1F;
    int g6 = (pixel >> 5) & 0x3F;
    int b5 = pixel & 0x1F;

    int r = r5 * 255 / 31;
    int g = g6 * 255 / 63;
    int b = b5 * 255 / 31;

    printf("bottom-center pixel(%d,%d) RGB: R=%d G=%d B=%d\r\n", x, y, r, g, b);
}
