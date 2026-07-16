#ifndef START_LINE_HPP
#define START_LINE_HPP

#include <stdint.h>

// 斑马线/停车线检测模块。
// 设计目标：
// 1. 用当前 RGB565 图像寻找赛道内的多个黑色块，模拟 Apex 的停车线检测思路。
// 2. 每经过一条停车线只计数一次，避免同一条线连续多帧重复计数。
// 3. 默认第 1 次停车线就触发停车；以后可以把 stop_target 改成 N，实现跑 N 圈再停。

// 清空停车线计数和锁存状态，适合重新开始比赛前调用。
void startline_reset(void);

// 已经识别到的停车线次数。
int startline_get_count(void);

// 最近一帧统计到的有效黑色块数量，用于调试阈值。
int startline_get_last_block_count(void);

// 设置第几次停车线才真正触发停车，最小 1，最大 20。
void startline_set_stop_target(int target_count);
int startline_get_stop_target(void);

// 输入一帧 RGB565 图像并更新停车线状态。
// 返回 true 表示停车线计数已经达到 stop_target，主循环应当停车。
bool startline_update_from_rgb565(const uint16_t *rgb565, int width, int height);

#endif
