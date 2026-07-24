#ifndef AVOID_HPP
#define AVOID_HPP

// 绕行总开关：0=只鸣笛不绕行，1=启用绕行状态机。
#define AVOID_MODE 1

/* 绕行状态机初始化。当前默认关闭绕行功能，只在启用后通过 Trans_line 挪动目标中线。 */
void avoid_init(void);

/* 绕行功能开关。true 允许视觉结果触发中线偏移，false 完全关闭绕行。 */
void avoid_set_enabled(bool enable);
bool avoid_is_enabled(void);

#define AVOID_PARAM_COUNT 6

/* Menu tuning interface: index 0~4 maps to avoid.cpp parameters. */
const char *avoid_get_param_name(int index);
float avoid_get_param_value(int index);
void avoid_set_param_value(int index, float value);
void avoid_adjust_param(int index, int direction);

/* 外部传入视觉识别结果。绕行开启时 result=0 左绕，result=1 右绕，其他值不触发。 */
void avoid_set_vision_result(int result);
bool avoid_should_slow_for_target(void);
/* 返回绕行预识别限速：-1=不限速，0=急刹，正数=限速。 */
int avoid_get_speed_limit(void);
void avoid_force_start(void);

/* 绕行专用周期任务。由独立 PIT 中断调用，负责推进绕行状态机。 */
void avoid_update_control(void);

/* 兼容旧接口：当前不接管左右轮目标速度，只通过 Trans_line 影响中线。 */
bool avoid_is_controlling(void);

/* 当前绕行目标中线偏移量。Nova 图像中心是 40，实际目标为 40 + Trans_line。 */
float avoid_get_trans_line(void);

/* 兼容旧接口：当前始终返回 false，继续普通巡线。 */
bool avoid_control_left(void);

/* 兼容旧接口：当前始终返回 false，继续普通巡线。 */
bool avoid_control_right(void);

/* 调试用：读取当前绕行状态。 */
int avoid_get_state(void);

#endif
