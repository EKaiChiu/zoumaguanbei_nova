#ifndef CAR_VISION_HPP
#define CAR_VISION_HPP
#include <cstdint>
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化视觉模块
 * 
 * 加载模型参数、权重文件和标签文件
 * 预计算 RGB565 转 BGR 的查找表
 * 
 * @return true 初始化成功
 * @return false 初始化失败（模型文件不存在等）
 */
bool vision_init();

/**
 * @brief 接收 RGB565 图像数据并进行识别
 * 
 * 这是电控调用的核心接口，传入摄像头采集的 RGB565 格式图像
 * 内部流程：RGB565→BGR 转换 → 红框检测 → ROI 裁剪 → ncnn 推理 → 类别映射
 * 
 * @param rgb565 RGB565 格式的图像数据指针
 * @param width 图像宽度
 * @param height 图像高度
 * @return int 识别结果（0=武器, 1=补给, 2=车辆, -1=未检测到目标或置信度不足）
 */
int vision_get_from_rgb565(const uint16_t *rgb565, int width, int height);

/**
 * @brief 关闭视觉模块
 * 
 * 释放模型资源和内存
 * 
 * @return void
 */
void vision_close();

#ifdef __cplusplus
}
#endif

#endif