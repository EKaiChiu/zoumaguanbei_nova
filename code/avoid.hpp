#ifndef AVOID_HPP
#define AVOID_HPP

// 绕行总开关：0=只鸣笛不绕行，1=启用绕行状态机。
#define AVOID_MODE 0

/* 绕行示例初始化。当前默认关闭绕行功能，避免识别到 0 后直接接管电机。 */
void avoid_init(void);

/* 绕行功能开关。true 允许视觉结果 0 触发绕行，false 关闭绕行。 */
void avoid_set_enabled(bool enable);
bool avoid_is_enabled(void);

/* 外部传入视觉识别结果。当前示例在绕行开启且 result 为 0/1/2 时触发。 */
void avoid_set_vision_result(int result);
void avoid_force_start(void);

/* 绕行专用周期任务。由独立 PIT 中断调用，负责推进绕行状态机。 */
void avoid_update_control(void);

/* 电机层读取：true 表示绕行正在接管左右轮目标速度。 */
bool avoid_is_controlling(void);

/* 当前绕行目标中线偏移量。Nova 图像中心是 40，实际目标为 40 + Trans_line。 */
float avoid_get_trans_line(void);

/* 绕行状态机主函数（左绕行）。true 表示接管电机，false 表示继续普通巡线。 */
bool avoid_control_left(void);

/* 绕行状态机主函数（右绕行，与左绕行镜像）。true 表示接管电机，false 表示继续普通巡线。 */
bool avoid_control_right(void);

/* 调试用：读取当前绕行状态。 */
int avoid_get_state(void);

#endif
