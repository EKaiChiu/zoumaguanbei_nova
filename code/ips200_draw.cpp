// ips200_draw.cpp
/*
 * IPS200屏幕划线功能 - 参考"雁过留痕圆环移植版本"tft180.c
 *
 * 核心逻辑：
 *   1. 显示二值化图像（80×60 → IPS200 160×120，2倍放大）
 *   2. 在图像上叠加边界线和中线（坐标×2映射）
 *
 * 与参考工程的对应关系：
 *   drawleftline()     → 左边界线（红色）
 *   drawrightline()    → 右边界线（红色）
 *   drawcenterline()   → 中线轨迹（蓝色）
 *   drawoffline()      → 丢线位置（红色横线）
 *   drawtowpointUP()   → 前瞻点上界（青色横线）
 *   drawtowpointDOWN() → 前瞻点下界（青色横线）
 */

#include "ips200_draw.hpp"
#include "image.hpp"
#include "red_detect.hpp"

// ════════════════════════════════════════════════
// 全局变量声明
// ════════════════════════════════════════════════
extern uint8 image_copy[LCDH][LCDW];

// ════════════════════════════════════════════════
// 功能 #1：画左边界线（红色）
//     数据源：ImageDeal[i].LeftBorder
// ════════════════════════════════════════════════
void draw_left_line(void)
{
    int x, i;
    for (i = 0; i < LCDH; i++) // 遍历所有行(0-59)
    {
        x = ImageDeal[i].LeftBorder * 2; // 坐标×2放大到160宽屏幕

        if (x >= 0 && x < 160) // 边界检查防止越界
        {
            ips200.draw_point(x, i * 2, RGB565_RED);     // 🔴红色左边界
            ips200.draw_point(x, i * 2 + 1, RGB565_RED); // 2倍放大填充（Y方向）
        }
    }
}

// ════════════════════════════════════════════════
// 功能 #2：画右边界线（🔴红色）
//     参考工程：drawrightline()
//     数据源：ImageDeal[i].RightBorder
// ════════════════════════════════════════════════
void draw_right_line(void)
{
    int x, i;
    for (i = 0; i < LCDH; i++)
    {
        x = ImageDeal[i].RightBorder * 2;

        if (x >= 0 && x < 160)
        {
            ips200.draw_point(x, i * 2, RGB565_RED);     // 🔴红色右边界
            ips200.draw_point(x, i * 2 + 1, RGB565_RED); // 2倍放大
        }
    }
}

// ════════════════════════════════════════════════
// 功能 #3：画中线轨迹线（🔵蓝色）
//     参考工程：drawcenterline()
//     ✅ 正确公式：(LeftBorder + RightBorder) / 2
//     或直接使用 ImageDeal[i].Center 字段
// ════════════════════════════════════════════════
void draw_center_line(void)
{
    int x, i;
    for (i = 0; i < LCDH - 1; i++) // 遍历除最后一行外的所有行
    {
        // ✅ 正确算法：中线 = (左边界 + 右边界) / 2
        x = (ImageDeal[i].LeftBorder + ImageDeal[i].RightBorder) / 2 * 2; // ×2放大到160宽

        if (x >= 0 && x < 160)
        {
            ips200.draw_point(x, i * 2, RGB565_BLUE);     // 🔵蓝色中线轨迹
            ips200.draw_point(x, i * 2 + 1, RGB565_BLUE); // 2倍放大
        }
    }
}

// ════════════════════════════════════════════════
// 功能 #4：画丢线位置标记（🔴红色水平横线）
//     参考工程：drawoffline()
// ════════════════════════════════════════════════
void draw_offline(void)
{
    int x, i;
    for (i = 0; i < LCDW; i++) // 横跨整个屏幕宽度(80像素)
    {
        x = ImageStatus.OFFLine * 2; // OFFLine行号×2放大（60→120）

        if (x >= 0 && x < 120) // Y坐标边界检查
        {
            ips200.draw_point(i * 2, x, RGB565_RED);     // 🔴红色横线
            ips200.draw_point(i * 2 + 1, x, RGB565_RED); // 2倍放大填充X方向
        }
    }
}

// ════════════════════════════════════════════════
// 功能 #5：画前瞻点上界（🟢青色水平横线）
//     参考工程：drawtowpointUP()
//     位置：TowPoint-5 行（前瞻点上方5像素）
// ════════════════════════════════════════════════
void draw_towpoint_up(void)
{
    int x, i;
    for (i = 0; i < LCDW; i++) // 遍历屏幕宽度(80像素)
    {
        x = (ImageStatus.TowPoint - 5) * 2; // 前瞻点上界 ×2放大到120高

        if (x >= 0 && x < 120)
        {
            ips200.draw_point(i * 2, x, RGB565_CYAN);     // 🟢青色横线
            ips200.draw_point(i * 2 + 1, x, RGB565_CYAN); // 2倍放大
        }
    }
}

// ════════════════════════════════════════════════
// 功能 #6：画前瞻点下界（🟢青色水平横线）
//     参考工程：drawtowpointDOWN()
//     位置：TowPoint+5 行（前瞻点下方5像素）
// ════════════════════════════════════════════════
void draw_towpoint_down(void)
{
    int x, i;
    for (i = 0; i < LCDW; i++)
    {
        x = (ImageStatus.TowPoint + 5) * 2; // 前瞻点下界 ×2放大到120高

        if (x >= 0 && x < 120)
        {
            ips200.draw_point(i * 2, x, RGB565_CYAN);     // 🟢青色横线
            ips200.draw_point(i * 2 + 1, x, RGB565_CYAN); // 2倍放大
        }
    }
}
// ════════════════════════════════════════════════
// 功能 #7：画红色目标边框（🩵青色矩形框）
//     数据源：red_detect 结构体的四个边界坐标
//     坐标已在160×120空间，无需放大
// ════════════════════════════════════════════════
void draw_red_target_on_screen(uint16 screen_buf[][160], red_detect *result)
{
    if (result == NULL || !result->is_found)
        return;

    int left = result->red_left_bound;
    int right = result->red_right_bound;
    int top = result->red_upper_bound;
    int bottom = result->red_lower_bound;

    // 夹紧到屏幕范围
    if (left < 0)
        left = 0;
    if (right > 159)
        right = 159;
    if (top < 0)
        top = 0;
    if (bottom > 119)
        bottom = 119;
    if (left >= right || top >= bottom)
        return;

    // 上边 + 下边（2像素宽）
    for (int x = left; x <= right; x++)
    {
        screen_buf[top][x] = RGB565_CYAN;
        screen_buf[top + 1][x] = RGB565_CYAN;
        screen_buf[bottom][x] = RGB565_CYAN;
        screen_buf[bottom - 1][x] = RGB565_CYAN;
    }

    // 左边 + 右边（2像素宽）
    for (int y = top + 2; y <= bottom - 2; y++)
    {
        screen_buf[y][left] = RGB565_CYAN;
        screen_buf[y][left + 1] = RGB565_CYAN;
        screen_buf[y][right] = RGB565_CYAN;
        screen_buf[y][right - 1] = RGB565_CYAN;
    }

    // 中心十字标记
    int cx = result->center_x;
    int cy = result->center_y;
    for (int dx = -3; dx <= 3; dx++)
    {
        int nx = cx + dx;
        if (nx >= 0 && nx < 160)
            screen_buf[cy][nx] = RGB565_CYAN;
    }
    for (int dy = -3; dy <= 3; dy++)
    {
        int ny = cy + dy;
        if (ny >= 0 && ny < 120)
            screen_buf[ny][cx] = RGB565_CYAN;
    }
}

/*
函数名称：void ips200_screen_display(void)
功能说明：IPS200屏幕显示（参考工程tft180风格）
参数说明：void
函数返回：void
修改时间：2026年6月19日
备注：功能未知
example：
*/
void ips200_screen_display(void)
{
    // ========== 第一步：显示二值化图像（参考工程：tft180_show_gray_image）==========
    ips200.show_gray_image(0, 0, Pixle[0], LCDW, LCDH, LCDW * 2, LCDH * 2, 1);

    // ========== 第二步：叠加所有标注线（与参考工程顺序一致）==========
    draw_left_line();                            // 🔴 左边界线（参考：drawleftline）
    draw_right_line();                           // 🔴 右边界线（参考：drawrightline）
    draw_center_line();                          // 🔵 中线轨迹（参考：drawcenterline）
    draw_offline();                              // 🔴 丢线位置（参考：drawoffline）
    draw_red_target_on_screen(nullptr, nullptr); // 🟢 红色目标标注（示例调用，实际使用时传入正确参数）
    // draw_towpoint_up();     // 🟢 前瞻点上界（参考：drawtowpointUP）
    // draw_towpoint_down();   // 🟢 前瞻点下界（参考：drawtowpointDOWN）
}

// 左右边界线：左蓝右黄，数据源：ImageDeal[y].LeftBorder / RightBorder
void draw_boundary_on_screen(uint16 screen_buf[][160])
{
    const uint16 LEFT_COLOR = 0x001F;  // 蓝
    const uint16 RIGHT_COLOR = 0xFFE0; // 黄

    for (int y = 0; y < LCDH; y++)
    {
        int left_x = ImageDeal[y].LeftBorder * 2;
        int right_x = ImageDeal[y].RightBorder * 2;
        int map_y = y * 2;

        if (left_x >= 2 && left_x < 158)
        {
            screen_buf[map_y][left_x] = LEFT_COLOR;
            screen_buf[map_y][left_x + 1] = LEFT_COLOR;
            screen_buf[map_y + 1][left_x] = LEFT_COLOR;
            screen_buf[map_y + 1][left_x + 1] = LEFT_COLOR;
        }

        if (right_x >= 2 && right_x < 158)
        {
            screen_buf[map_y][right_x] = RIGHT_COLOR;
            screen_buf[map_y][right_x + 1] = RIGHT_COLOR;
            screen_buf[map_y + 1][right_x] = RIGHT_COLOR;
            screen_buf[map_y + 1][right_x + 1] = RIGHT_COLOR;
        }
    }
}

// 中线轨迹（红色）
void draw_trajectory_on_screen(uint16 screen_buf[][160])
{
    const uint16 RED_COLOR = 0xF800;

    for (int y = 0; y < LCDH; y++)
    {
        int mid_x = (ImageDeal[y].LeftBorder + ImageDeal[y].RightBorder) / 2 * 2;
        int map_y = y * 2;

        if (mid_x >= 2 && mid_x < 158)
        {
            screen_buf[map_y][mid_x] = RED_COLOR;
            screen_buf[map_y][mid_x + 1] = RED_COLOR;
            screen_buf[map_y + 1][mid_x] = RED_COLOR;
            screen_buf[map_y + 1][mid_x + 1] = RED_COLOR;
        }
    }
}

// 丢线位置横线（红色），标记 OFFLine 行
void draw_offline_line_on_screen(uint16 screen_buf[][160])
{
    const uint16 RED_COLOR = 0xF800;
    int offline_y = ImageStatus.OFFLine * 2;

    if (offline_y >= 0 && offline_y < 120)
    {
        for (int x = 0; x < 160; x++)
            screen_buf[offline_y][x] = RED_COLOR;
    }
}

// ════════════════════════════════════════════════════════════
// ⭐⭐ 高级功能 #8.6：在RGB565缓冲区绘制🟢前瞻点范围横线
//
// 参考工程：drawtowpointUP() + drawtowpointDOWN()
// 功能：在TowPoint±5行的位置画两条青色横线，标记前瞻点搜索范围
// 参数：screen_buf - 160×120的RGB565显存数组
// 颜色：RGB565青色(0x07FF)
// ════════════════════════════════════════════════════════════
void draw_towpoint_lines_on_screen(uint16 screen_buf[][160])
{
    const uint16 CYAN_COLOR = 0x07FF; // RGB565青色: R=0,G=63,B=31

    int towpoint_up_y = (ImageStatus.TowPoint - 5) * 2;   // 上界：TowPoint-5 → ×2
    int towpoint_down_y = (ImageStatus.TowPoint + 5) * 2; // 下界：TowPoint+5 → ×2

    // 🟢 上界横线（TowPoint-5）
    if (towpoint_up_y >= 0 && towpoint_up_y < 120)
    {
        for (int x = 0; x < 160; x++)
        {
            screen_buf[towpoint_up_y][x] = CYAN_COLOR; // 🟢青色上界
        }
    }

    // 🟢 下界横线（TowPoint+5）
    if (towpoint_down_y >= 0 && towpoint_down_y < 120)
    {
        for (int x = 0; x < 160; x++)
        {
            screen_buf[towpoint_down_y][x] = CYAN_COLOR; // 🟢青色下界
        }
    }
}

// ════════════════════════════════════════════════
// ⭐⭐ 图传标注功能 #9：为逐飞助手的image_copy添加标注线
//
// 参考工程：drawleftline() + drawrightline() + drawcenterline()
//           + drawoffline() + drawtowpointUP() + drawtowpointDOWN()
// 数据源：ImageDeal[i].LeftBorder / RightBorder（与参考工程一致）
// 功能：在灰度二值化图上叠加浅色标注，便于电脑端分析
// 目标数组：全局变量 image_copy[UVC_HEIGHT][UVC_WIDTH]
// 灰度颜色方案：
//   边界线 = 深灰色180（在白路面上清晰可见）
//   轨迹线 = 中灰色120（明显区别于白路面255和黑背景0）
//   丢线/前瞻点 = 浅灰色200（最显眼，用于重要标记）
// ════════════════════════════════════════════════
void draw_annotation_on_imagecopy(void)
{
    const uint8 BOUNDARY_GRAY = 180;   // 边界线：深灰色
    const uint8 TRAJECTORY_GRAY = 120; // 轨迹线：中灰色
    const uint8 MARKER_GRAY = 200;     // 标记线：浅灰色（丢线+前瞻点）

    // ========== 第一部分：边界线和中线（遍历所有行）==========
    for (int y = 0; y < LCDH; y++)
    {
        // ⚠️ 使用 LeftBorder/RightBorder（参考工程风格）
        int left_x = ImageDeal[y].LeftBorder;
        int right_x = ImageDeal[y].RightBorder;
        // ⚠️ 参考工程算法：中线 = (左边界 + 右边界) / 2（正确✅）
        int mid_x = (ImageDeal[y].LeftBorder + ImageDeal[y].RightBorder) / 2;

        // 左边界线
        if (left_x >= 0 && left_x < LCDW)
        {
            image_copy[y][left_x] = BOUNDARY_GRAY;
        }

        // 右边界线
        if (right_x >= 0 && right_x < LCDW)
        {
            image_copy[y][right_x] = BOUNDARY_GRAY;
        }

        // 中线轨迹（3像素宽：左-中-右对称）
        if (mid_x >= 1 && mid_x < (LCDW - 1))
        {
            image_copy[y][mid_x] = TRAJECTORY_GRAY;
            if (mid_x - 1 >= 0)
            { // 左侧1像素
                image_copy[y][mid_x - 1] = TRAJECTORY_GRAY;
            }
            if (mid_x + 1 < LCDW)
            { // 右侧1像素
                image_copy[y][mid_x + 1] = TRAJECTORY_GRAY;
            }
        }
    }

    // // ========== 第二部分：丢线横线（参考工程：drawoffline）==========
    // int offline_y = ImageStatus.OFFLine * 2;
    // if(offline_y >= 0 && offline_y < 120)
    // {
    //     for(int i = 0; i < 160; i++)
    //     {
    //         image_copy[offline_y][i] = MARKER_GRAY;  // 浅灰色丢线标记
    //     }
    // }

    // // ========== 第三部分：前瞻点上下界横线（参考工程：drawtowpointUP/DOWN）==========
    // int towpoint_up_y = (ImageStatus.TowPoint - 5) * 2;    // 上界：TowPoint-5
    // int towpoint_down_y = (ImageStatus.TowPoint + 5) * 2;  // 下界：TowPoint+5

    // // 上界横线
    // if(towpoint_up_y >= 0 && towpoint_up_y < 120)
    // {
    //     for(int i = 0; i < 160; i++)
    //     {
    //         image_copy[towpoint_up_y][i] = MARKER_GRAY;  // 浅灰色前瞻点上界
    //     }
    // }

    // // 下界横线
    // if(towpoint_down_y >= 0 && towpoint_down_y < 120)
    // {
    //     for(int i = 0; i < 160; i++)
    //     {
    //         image_copy[towpoint_down_y][i] = MARKER_GRAY;  // 浅灰色前瞻点下界
    //     }
    // }
}
