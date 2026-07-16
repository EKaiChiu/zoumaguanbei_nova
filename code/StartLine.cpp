#include "StartLine.hpp"
#include "image.hpp"
#include "beep.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <cstdio>

// HSV 黑色阈值里的 V 最大值。数值越大，越容易把暗色区域当成黑块。
static const int STARTLINE_BLACK_V_MAX = 60;
// 单个黑色连通块的最小面积。太小会把噪点算进去，太大会漏检远处小黑块。
static const int STARTLINE_MIN_AREA = 30;
// 一帧里至少需要多少个有效黑块，才认为看到了斑马线/停车线。
static const int STARTLINE_MIN_BLOCKS = 5;
// 识别到一条停车线后，需要连续多少帧看不到停车线，才允许下一次计数。
static const int STARTLINE_RELEASE_FRAMES = 8;

// 已经通过的停车线数量。
static int startline_count = 0;
// 第几次停车线才真正触发停车。默认 1，后面跑多圈时可以改大。
static int startline_stop_target = 1;
// 最近一帧统计到的黑块数量，方便串口/屏幕调试。
static int last_black_block_count = 0;
// 锁存标志：同一条停车线连续出现在视野里时，只允许计数一次。
static bool startline_latched = false;
// 解锁计数：离开停车线一段时间后，解除锁存，允许下一条停车线计数。
static int startline_release_count = 0;

// 将摄像头 RGB565 原始图像转成 OpenCV BGR 图像，便于后续转 HSV。
static cv::Mat rgb565_to_bgr_mat(const uint16_t *rgb565, int width, int height)
{
    cv::Mat bgr(height, width, CV_8UC3);
    for (int y = 0; y < height; y++)
    {
        cv::Vec3b *row = bgr.ptr<cv::Vec3b>(y);
        for (int x = 0; x < width; x++)
        {
            uint16_t pixel = rgb565[y * width + x];
            uint8_t r = (uint8_t)(((pixel >> 11) & 0x1F) * 255 / 31);
            uint8_t g = (uint8_t)(((pixel >> 5) & 0x3F) * 255 / 63);
            uint8_t b = (uint8_t)((pixel & 0x1F) * 255 / 31);
            row[x] = cv::Vec3b(b, g, r);
        }
    }
    return bgr;
}

// 判断黑块中心是否在当前赛道左右边界之间。
// 这样可以过滤掉赛道外、屏幕边缘、背景里的黑色干扰。
static bool point_inside_track(int x, int y, int width, int height)
{
    if (width <= 0 || height <= 0)
        return false;

    // ImageDeal 是 80x60 巡线坐标，RGB 图像可能是 160x120 或 320x240，需要按比例映射行号。
    int image_y = y * LCDH / height;
    if (image_y < 0)
        image_y = 0;
    if (image_y >= LCDH)
        image_y = LCDH - 1;

    int left = ImageDeal[image_y].LeftBorder * width / LCDW;
    int right = ImageDeal[image_y].RightBorder * width / LCDW;
    if (right <= left + 6)
        return false;

    return x > left && x < right;
}

void startline_reset(void)
{
    startline_count = 0;
    last_black_block_count = 0;
    startline_latched = false;
    startline_release_count = 0;
}

int startline_get_count(void)
{
    return startline_count;
}

int startline_get_last_block_count(void)
{
    return last_black_block_count;
}

void startline_set_stop_target(int target_count)
{
    if (target_count < 1)
        target_count = 1;
    if (target_count > 20)
        target_count = 20;
    startline_stop_target = target_count;
}

int startline_get_stop_target(void)
{
    return startline_stop_target;
}

bool startline_update_from_rgb565(const uint16_t *rgb565, int width, int height)
{
    if (rgb565 == nullptr || width <= 0 || height <= 0)
        return false;

    cv::Mat bgr = rgb565_to_bgr_mat(rgb565, width, height);
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // 只按黑色检测：H 不限制，S 不限制，V 足够低就认为是黑色候选。
    cv::Mat black_mask;
    cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, STARTLINE_BLACK_V_MAX), black_mask);

    // 连通域统计，每个黑块会得到面积和中心点。
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(black_mask, labels, stats, centroids, 8, CV_32S);

    int black_block_count = 0;
    for (int label = 1; label < num_labels; label++)
    {
        int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < STARTLINE_MIN_AREA)
            continue;

        int cx = (int)(centroids.at<double>(label, 0) + 0.5);
        int cy = (int)(centroids.at<double>(label, 1) + 0.5);

        // 过滤最上方区域，防止远处噪点、屏幕边缘、背景阴影参与判断。
        if (cy < height / 4 || cy >= height)
            continue;
        if (!point_inside_track(cx, cy, width, height))
            continue;

        black_block_count++;
    }

    last_black_block_count = black_block_count;
    bool detected = black_block_count >= STARTLINE_MIN_BLOCKS;

    // 没检测到停车线时，如果之前锁存过，就累计释放帧数。
    // 连续离开停车线 STARTLINE_RELEASE_FRAMES 帧后，下一条停车线才可以再次计数。
    if (!detected)
    {
        if (startline_latched)
        {
            startline_release_count++;
            if (startline_release_count >= STARTLINE_RELEASE_FRAMES)
            {
                startline_latched = false;
                startline_release_count = 0;
            }
        }
        return false;
    }

    // 已经锁存时说明还是同一条停车线，不重复计数。
    startline_release_count = 0;
    if (startline_latched)
        return false;

    // 新的一条停车线：计数加一，并判断是否达到停车目标。
    startline_latched = true;
    beep_short();
    startline_count++;
    printf("[STARTLINE] count=%d target=%d blocks=%d\r\n", startline_count, startline_stop_target, black_block_count);

    return startline_count >= startline_stop_target;
}
