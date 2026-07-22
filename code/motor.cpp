#include "motor.hpp"
#include "avoid.hpp"
#include "config.hpp"
zf_driver_gpio motor1_gpio(motor1_dir, O_RDWR);
zf_driver_gpio motor2_gpio(motor2_dir, O_RDWR);

zf_driver_pwm motor1_pwm_1(motor1_pwm);
zf_driver_pwm motor2_pwm_2(motor2_pwm);

zf_driver_encoder encoder_dir_1(encoderA);
zf_driver_encoder encoder_dir_2(encoderB);

struct pwm_info motor1_pwm_info;
struct pwm_info motor2_pwm_info;

// 编码器计数值
int16 encoderA_count = 0; // 左轮
int16 encoderB_count = 0; // 右轮

float speed_p_l, speed_i_l, speed_d_l;
float speed_error_l;
float speed_goal_l;
float speed_pid_out_l;
float speed_integral_l;
float speed_last_error_l;
static float speed_dout_filtered_l = 1.0f;

float speed_p_r, speed_i_r, speed_d_r;
float speed_error_r;
float speed_goal_r;
float speed_pid_out_r;
float speed_integral_r;
float speed_last_error_r;
static float speed_dout_filtered_r = 1.0f;

// 测试模式：0=寻迹, 1=速度测试, 2=固定PWM, 3=编码器观察, 4=映射验证, 6=手动PID调速
int test_mode = 0;
int mode2_pwm = 1200;

int pwm_max = 3500, pwm_min = -3500; // PWM限幅（duty=800对应8%占空比）

int16 speed_begin = 500;
int16 speed_expect = 0;

float diff_speed_error;
int16 diff_speedl_expect, diff_speedr_expect;

float diff_kp = 2.6f, diff_kd = 3.5f; // 差速PID参数（快速响应）
static float turn_kp_low = 1.82f;
static float turn_kp_mid = 5.0f;
static float turn_kp_big = 6.0f;
static float turn_kp_sharp = 8.0f;
static float turn_ring_multiplier = 1.50f;
static int sharp_turn_print_cnt = 0;
static bool uphill_boost_allowed = false;
float k = 2.868;
float dif = 0;

float k1 = 0.6;
float y, x;

// 测试参数
int test_wheel = 1;  // 测试轮选择
int test_speed = 60; // 测试目标速度

int speed_pwm_min_l = 780;
int speed_pwm_min_r = 780;
float speed_pwm_feedforward_l = 6.2f;
float speed_pwm_feedforward_r = 5.8f;
static int line_base_speed = 160;
static float speed_ratio_ring = 0.90f;
static float speed_ratio_small = 0.80f;
static float speed_ratio_mid = 0.68f;
static float speed_ratio_big = 0.55f;
static float speed_ratio_sharp = 0.45f;
static int speed_expect_limit = 500;

static void reset_speed_pid_state()
{
    speed_error_l = 0;
    speed_integral_l = 0;
    speed_last_error_l = 0;
    speed_pid_out_l = 0;
    speed_dout_filtered_l = 0;

    speed_error_r = 0;
    speed_integral_r = 0;
    speed_last_error_r = 0;
    speed_pid_out_r = 0;
    speed_dout_filtered_r = 0;
}

static float abs_float(float value)
{
    return value < 0.0f ? -value : value;
}

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

void motor_set_line_base_speed(int speed)
{
    if (speed < 45)
        speed = 45;
    if (speed > 350)
        speed = 350;
    line_base_speed = speed;
}

int motor_get_line_base_speed(void)
{
    return line_base_speed;
}

static float clamp_speed_ratio(float value)
{
    if (value < 0.1f)
        return 0.1f;
    if (value > 1.5f)
        return 1.5f;
    return value;
}

static int clamp_speed_limit(int value)
{
    if (value < 100)
        return 100;
    if (value > 800)
        return 800;
    return value;
}

const char *motor_get_speed_param_name(int index)
{
    switch (index)
    {
        case 0:
            return "Base";
        case 1:
            return "Ring";
        case 2:
            return "Small";
        case 3:
            return "Mid";
        case 4:
            return "Big";
        case 5:
            return "Sharp";
        case 6:
            return "Limit";
        default:
            return "Unknown";
    }
}

float motor_get_speed_param_value(int index)
{
    switch (index)
    {
        case 0:
            return (float)line_base_speed;
        case 1:
            return speed_ratio_ring;
        case 2:
            return speed_ratio_small;
        case 3:
            return speed_ratio_mid;
        case 4:
            return speed_ratio_big;
        case 5:
            return speed_ratio_sharp;
        case 6:
            return (float)speed_expect_limit;
        default:
            return 0.0f;
    }
}

void motor_set_speed_param_value(int index, float value)
{
    switch (index)
    {
        case 0:
            motor_set_line_base_speed((int)(value + 0.5f));
            break;
        case 1:
            speed_ratio_ring = clamp_speed_ratio(value);
            break;
        case 2:
            speed_ratio_small = clamp_speed_ratio(value);
            break;
        case 3:
            speed_ratio_mid = clamp_speed_ratio(value);
            break;
        case 4:
            speed_ratio_big = clamp_speed_ratio(value);
            break;
        case 5:
            speed_ratio_sharp = clamp_speed_ratio(value);
            break;
        case 6:
            speed_expect_limit = clamp_speed_limit((int)(value + 0.5f));
            break;
        default:
            break;
    }
}

void motor_adjust_speed_param(int index, int direction)
{
    if (index == 0)
    {
        motor_set_line_base_speed(line_base_speed + direction * 5);
        return;
    }
    if (index == 6)
    {
        motor_set_speed_param_value(index, (float)(speed_expect_limit + direction * 50));
        return;
    }

    motor_set_speed_param_value(index, motor_get_speed_param_value(index) + direction * 0.1f);
}

static float clamp_turn_param(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

const char *motor_get_turn_param_name(int index)
{
    switch (index)
    {
        case 0:
            return "LowKp";
        case 1:
            return "MidKp";
        case 2:
            return "BigKp";
        case 3:
            return "SharpKp";
        case 4:
            return "RingMul";
        default:
            return "Unknown";
    }
}

float motor_get_turn_param_value(int index)
{
    switch (index)
    {
        case 0:
            return turn_kp_low;
        case 1:
            return turn_kp_mid;
        case 2:
            return turn_kp_big;
        case 3:
            return turn_kp_sharp;
        case 4:
            return turn_ring_multiplier;
        default:
            return 0.0f;
    }
}

void motor_set_turn_param_value(int index, float value)
{
    switch (index)
    {
        case 0:
            turn_kp_low = clamp_turn_param(value, 0.0f, 20.0f);
            break;
        case 1:
            turn_kp_mid = clamp_turn_param(value, 0.0f, 20.0f);
            break;
        case 2:
            turn_kp_big = clamp_turn_param(value, 0.0f, 20.0f);
            break;
        case 3:
            turn_kp_sharp = clamp_turn_param(value, 0.0f, 20.0f);
            break;
        case 4:
            turn_ring_multiplier = clamp_turn_param(value, 0.1f, 3.0f);
            break;
        default:
            break;
    }
}

void motor_adjust_turn_param(int index, int direction)
{
    if (direction == 0)
        return;
    float step = 0.1f;
    motor_set_turn_param_value(index, motor_get_turn_param_value(index) + (direction > 0 ? step : -step));
}

static void setup_speed_test_target()
{
    if (test_wheel == 1)
    {
        diff_speedl_expect = test_speed;
        diff_speedr_expect = 0;
    }
    else if (test_wheel == 2)
    {
        diff_speedl_expect = 0;
        diff_speedr_expect = test_speed;
    }
    else
    {
        diff_speedl_expect = test_speed;
        diff_speedr_expect = test_speed;
    }

    speed_goal_l = (float)diff_speedl_expect;
    speed_goal_r = (float)diff_speedr_expect;
}

static int calc_speed_pwm_debug(float goal, float comp, float feedforward, int min_pwm)
{
    int final_pwm = (int)(goal * feedforward + comp);

    if (final_pwm > pwm_max)
        final_pwm = pwm_max;
    if (final_pwm < pwm_min)
        final_pwm = pwm_min;

    if (goal == 0.0f)
        return 0;

    if (goal > 0.0f)
    {
        if (final_pwm < 0)
            final_pwm = 0;
        else if (final_pwm > 0 && final_pwm < min_pwm)
            final_pwm = min_pwm;
    }

    return final_pwm;
}

void motor_argument()
{
    // ✅ 慢速目标：100 pulse/周期（对应duty≈800，8%占空比）
    speed_goal_l = 120.0f;
    speed_goal_r = 120.0f;

    diff_speedl_expect = (int16)speed_goal_l;
    diff_speedr_expect = (int16)speed_goal_r;

    // 速度环 PID：模式5实车扫描得到的巡线中高速折中参数（候选6）。
    speed_p_l = 0.340f;
    speed_i_l = 0.008f;
    speed_d_l = 0.180f;

    speed_p_r = 0.260f;
    speed_i_r = 0.006f;
    speed_d_r = 0.080f;
}

// 临时诊断函数声明（调试完后删除！）
void debug_print_centers();

typedef struct
{
    float sum_speed;
    float sum_error;
    float sum_abs_error;
    float max_speed;
    float max_abs_error;
    float last_error;
    int sign_changes;
    int samples;
} SpeedTuneStats;

static volatile int mode5_manual_pending = 0;
static int mode5_manual_mode = 0;
static int mode5_auto_done = 0;
static float mode5_cmd_kp_l = 0.25f;
static float mode5_cmd_ki_l = 0.004f;
static float mode5_cmd_kd_l = 0.35f;
static float mode5_cmd_ff_l = 7.0f;
static int mode5_cmd_min_l = 1250;
static float mode5_cmd_kp_r = 0.25f;
static float mode5_cmd_ki_r = 0.004f;
static float mode5_cmd_kd_r = 0.35f;
static float mode5_cmd_ff_r = 7.0f;
static int mode5_cmd_min_r = 1250;
static int mode5_cmd_target = 160;

static float mode5_best_score_l = 999999.0f;
static float mode5_best_score_r = 999999.0f;
static float mode5_best_kp_l = 0.25f, mode5_best_ki_l = 0.004f, mode5_best_kd_l = 0.35f, mode5_best_ff_l = 7.0f;
static int mode5_best_min_l = 1250;
static float mode5_best_kp_r = 0.25f, mode5_best_ki_r = 0.004f, mode5_best_kd_r = 0.35f, mode5_best_ff_r = 7.0f;
static int mode5_best_min_r = 1250;

static void mode5_apply_manual_command()
{
    speed_p_l = clamp_float(mode5_cmd_kp_l, 0.0f, 5.0f);
    speed_i_l = clamp_float(mode5_cmd_ki_l, 0.0f, 0.20f);
    speed_d_l = clamp_float(mode5_cmd_kd_l, 0.0f, 1.00f);
    speed_pwm_feedforward_l = clamp_float(mode5_cmd_ff_l, 0.0f, 30.0f);
    speed_pwm_min_l = (int)clamp_float((float)mode5_cmd_min_l, 0.0f, 2500.0f);

    speed_p_r = clamp_float(mode5_cmd_kp_r, 0.0f, 5.0f);
    speed_i_r = clamp_float(mode5_cmd_ki_r, 0.0f, 0.20f);
    speed_d_r = clamp_float(mode5_cmd_kd_r, 0.0f, 1.00f);
    speed_pwm_feedforward_r = clamp_float(mode5_cmd_ff_r, 0.0f, 30.0f);
    speed_pwm_min_r = (int)clamp_float((float)mode5_cmd_min_r, 0.0f, 2500.0f);
    test_speed = (int)clamp_float((float)mode5_cmd_target, 70.0f, 280.0f);
}

void mode5_set_manual_params(float kp, float ki, float kd, float feedforward, int min_pwm, int target_speed)
{
    mode5_cmd_kp_l = kp;
    mode5_cmd_ki_l = ki;
    mode5_cmd_kd_l = kd;
    mode5_cmd_ff_l = feedforward;
    mode5_cmd_min_l = min_pwm;
    mode5_cmd_kp_r = kp;
    mode5_cmd_ki_r = ki;
    mode5_cmd_kd_r = kd;
    mode5_cmd_ff_r = feedforward;
    mode5_cmd_min_r = min_pwm;
    mode5_cmd_target = target_speed;
    mode5_manual_pending = 1;
}

void mode5_set_manual_lr_params(float kp_l, float ki_l, float kd_l, float ff_l, int min_l, float kp_r, float ki_r,
                                float kd_r, float ff_r, int min_r, int target_speed)
{
    mode5_cmd_kp_l = kp_l;
    mode5_cmd_ki_l = ki_l;
    mode5_cmd_kd_l = kd_l;
    mode5_cmd_ff_l = ff_l;
    mode5_cmd_min_l = min_l;
    mode5_cmd_kp_r = kp_r;
    mode5_cmd_ki_r = ki_r;
    mode5_cmd_kd_r = kd_r;
    mode5_cmd_ff_r = ff_r;
    mode5_cmd_min_r = min_r;
    mode5_cmd_target = target_speed;
    mode5_manual_pending = 1;
}

static void speed_tune_stats_reset(SpeedTuneStats *stats)
{
    stats->sum_speed = 0.0f;
    stats->sum_error = 0.0f;
    stats->sum_abs_error = 0.0f;
    stats->max_speed = 0.0f;
    stats->max_abs_error = 0.0f;
    stats->last_error = 0.0f;
    stats->sign_changes = 0;
    stats->samples = 0;
}

static void speed_tune_stats_update(SpeedTuneStats *stats, float speed, float error)
{
    float abs_error = abs_float(error);

    if (stats->samples > 0 &&
        ((error > 0.0f && stats->last_error < 0.0f) || (error < 0.0f && stats->last_error > 0.0f)))
    {
        stats->sign_changes++;
    }

    stats->last_error = error;
    stats->sum_speed += speed;
    stats->sum_error += error;
    stats->sum_abs_error += abs_error;
    if (speed > stats->max_speed)
        stats->max_speed = speed;
    if (abs_error > stats->max_abs_error)
        stats->max_abs_error = abs_error;
    stats->samples++;
}

static void speed_tune_one_side(float target, float avg_speed, float avg_error, float avg_abs_error, float max_speed,
                                int sign_changes, float *kp, float *ki, float *kd, float *feedforward, int *min_pwm)
{
    float abs_target = abs_float(target);
    float overshoot = 0.0f;
    if (abs_target > 1.0f)
        overshoot = (max_speed - abs_target) / abs_target;

    if (avg_speed < abs_target * 0.75f)
    {
        *min_pwm += 50;
        *feedforward += 0.45f;
        *kp += 0.08f;
    }
    else if (avg_error > abs_target * 0.10f)
    {
        *feedforward += 0.20f;
        *ki += 0.003f;
        *kp += 0.03f;
    }
    else if (avg_error < -abs_target * 0.10f)
    {
        *feedforward -= 0.20f;
        *ki -= 0.002f;
    }

    if (sign_changes > 5 || overshoot > 0.25f)
    {
        *kp *= 0.88f;
        *kd += 0.02f;
        *min_pwm -= 25;
    }
    else if (avg_abs_error > abs_target * 0.12f)
    {
        *kp += 0.04f;
    }

    *kp = clamp_float(*kp, 0.20f, 3.50f);
    *ki = clamp_float(*ki, 0.005f, 0.080f);
    *kd = clamp_float(*kd, 0.0f, 0.45f);
    *feedforward = clamp_float(*feedforward, 5.0f, 22.0f);
    *min_pwm = (int)clamp_float((float)(*min_pwm), 700.0f, 1800.0f);
}

static int mode6_target_speed = 120;
static int mode6_print_cnt = 0;

void motor_mode6_set_lr_params(float kp_l, float ki_l, float kd_l, float ff_l, int min_l, float kp_r, float ki_r,
                               float kd_r, float ff_r, int min_r, int target_speed)
{
    speed_p_l = clamp_float(kp_l, 0.0f, 5.0f);
    speed_i_l = clamp_float(ki_l, 0.0f, 0.20f);
    speed_d_l = clamp_float(kd_l, 0.0f, 1.00f);
    speed_pwm_feedforward_l = clamp_float(ff_l, 0.0f, 30.0f);
    speed_pwm_min_l = (int)clamp_float((float)min_l, 0.0f, 2500.0f);

    speed_p_r = clamp_float(kp_r, 0.0f, 5.0f);
    speed_i_r = clamp_float(ki_r, 0.0f, 0.20f);
    speed_d_r = clamp_float(kd_r, 0.0f, 1.00f);
    speed_pwm_feedforward_r = clamp_float(ff_r, 0.0f, 30.0f);
    speed_pwm_min_r = (int)clamp_float((float)min_r, 0.0f, 2500.0f);

    mode6_target_speed = (int)clamp_float((float)target_speed, 0.0f, 320.0f);
    reset_speed_pid_state();
    printf("[MODE6][PID] L kp=%.3f ki=%.4f kd=%.3f ff=%.1f min=%d | R kp=%.3f ki=%.4f kd=%.3f ff=%.1f min=%d "
           "target=%d\r\n",
           speed_p_l, speed_i_l, speed_d_l, speed_pwm_feedforward_l, speed_pwm_min_l, speed_p_r, speed_i_r, speed_d_r,
           speed_pwm_feedforward_r, speed_pwm_min_r, mode6_target_speed);
}

void motor_mode6_adjust_target(int direction)
{
    int step = 10;
    mode6_target_speed += direction * step;
    if (mode6_target_speed < 0)
        mode6_target_speed = 0;
    if (mode6_target_speed > 320)
        mode6_target_speed = 320;
    reset_speed_pid_state();
    printf("[MODE6][KEY] target=%d\r\n", mode6_target_speed);
}

int motor_mode6_get_target(void)
{
    return mode6_target_speed;
}

void motor_control()
{
    static int last_test_mode = -1;
    if (test_mode != last_test_mode)
    {
        reset_speed_pid_state();
        last_test_mode = test_mode;
    }

    // ════════════════════════════════════════════════
    // 1. 获取编码器数据（速度闭环：读完后清零！）
    // ════════════════════════════════════════════════
    encoderA_count = encoder_dir_2.get_count();  // 左轮Encoder2
    encoderB_count = -encoder_dir_1.get_count(); // 右轮Encoder1（取反）
    int16 encoder_raw_l = encoderA_count;
    int16 encoder_raw_r = encoderB_count;

    encoder_dir_2.clear_count(); // 清零！
    encoder_dir_1.clear_count(); // 清零！

    // ✅ 关键：同步差速环目标到PID目标（仅在寻迹模式）
    int16 encoder_swap_temp = encoderA_count;
    encoderA_count = encoderB_count;
    encoderB_count = encoder_swap_temp;

    // ════════════════════════════════════════════════
    // 模式4：编码器映射验证（修正后测试）
    // ════════════════════════════════════════════════
    if (test_mode == 4)
    {
        static int started = 0;
        static int cnt = 0;

        if (!started)
        {
            started = 1;
            printf("=== ENCODER MAPPING TEST ===\r\n");
            printf("Motor1(Left) -> Encoder2, Motor2(Right) -> Encoder1\r\n");
            printf("Expected: Both encoders > 0 (forward)\r\n");
        }

        // 固定PWM输出（8%占空比，慢速测试）
        motor1_gpio.set_level(0);
        motor1_pwm_1.set_duty(800); // 左轮duty=800（对应≈100 pulse/周期）
        motor2_gpio.set_level(0);
        motor2_pwm_2.set_duty(800); // 右轮duty=800

        // 每20帧打印一次
        if (++cnt % 20 == 0)
        {
            printf("L:enc=%d | R:enc=%d | Status:%s\r\n", encoderA_count, encoderB_count,
                   (encoderA_count > 0 && encoderB_count > 0) ? "✅ OK" : "❌ ERROR");
        }
        return;
    }

    // ════════════════════════════════════════════════
    // 模式1：编码器正反观察
    // ════════════════════════════════════════════════
    if (test_mode == 1)
    {
        static int cnt = 0;
        if (cnt < 3)
        {
            printf("[MODE1] Encoder raw data === Don't move! ===\r\n");
            cnt++;
        }
        printf("L:%d | R:%d\r\n", encoderA_count, encoderB_count);
        return;
    }
    // ════════════════════════════════════════════════
    // 模式2：固定PWM直行（硬件方向测试）
    // ════════════════════════════════════════════════
    if (test_mode == 2)
    {
        static int cnt = 0;
        if (cnt < 1)
        {
            printf("[MODE2] Fixed PWM L=R=duty(%d)\r\n", mode2_pwm);
            cnt++;
        }
        motor1_gpio.set_level(0);
        motor1_pwm_1.set_duty(mode2_pwm);
        motor2_gpio.set_level(0);
        motor2_pwm_2.set_duty(mode2_pwm);
        static int print_cnt = 0;
        if (++print_cnt >= 20)
        {
            print_cnt = 0;
            printf("L:%d R:%d\r\n", encoderA_count, encoderB_count);
        }
        return;
    }

    // ════════════════════════════════════════════════
    // 模式3：速度PID测试（单独测左/右轮）
    // ════════════════════════════════════════════════
    if (test_mode == 5)
    {
        struct Mode5Candidate
        {
            float kp_l, ki_l, kd_l, ff_l;
            int min_l;
            float kp_r, ki_r, kd_r, ff_r;
            int min_r;
        };

        static const int target_table[] = {80, 120, 160, 200, 240, 200, 160, 120};
        static const Mode5Candidate candidates[] = {
            {0.22f, 0.008f, 0.45f, 6.2f, 700, 0.20f, 0.006f, 0.45f, 5.0f, 700},
            {0.26f, 0.010f, 0.45f, 6.6f, 700, 0.20f, 0.008f, 0.45f, 5.2f, 700},
            {0.30f, 0.012f, 0.45f, 7.0f, 725, 0.22f, 0.010f, 0.45f, 5.5f, 700},
            {0.34f, 0.014f, 0.45f, 7.4f, 750, 0.24f, 0.012f, 0.45f, 5.8f, 725},
            {0.38f, 0.016f, 0.45f, 7.8f, 775, 0.26f, 0.014f, 0.45f, 6.1f, 750},
            {0.42f, 0.018f, 0.45f, 8.2f, 825, 0.30f, 0.016f, 0.45f, 6.5f, 800},
            {0.48f, 0.020f, 0.45f, 8.8f, 900, 0.36f, 0.018f, 0.45f, 7.0f, 850},
            {0.55f, 0.022f, 0.45f, 9.4f, 975, 0.42f, 0.020f, 0.45f, 7.6f, 900},
        };
        static const int target_count = (int)(sizeof(target_table) / sizeof(target_table[0]));
        static const int candidate_count = (int)(sizeof(candidates) / sizeof(candidates[0]));

        static int started = 0;
        static int candidate_index = 0;
        static int target_index = 0;
        static int settle_tick = 0;
        static int sample_tick = 0;
        static int printed_header = 0;
        static float candidate_score = 0.0f;
        static SpeedTuneStats stats_l;
        static SpeedTuneStats stats_r;

        if (mode5_manual_pending)
        {
            mode5_apply_manual_command();
            mode5_manual_pending = 0;
            reset_speed_pid_state();
            printf("[MODE5][MANUAL] L kp=%.3f ki=%.4f kd=%.3f ff=%.1f min=%d | R kp=%.3f ki=%.4f kd=%.3f ff=%.1f "
                   "min=%d base=%d\r\n",
                   speed_p_l, speed_i_l, speed_d_l, speed_pwm_feedforward_l, speed_pwm_min_l, speed_p_r, speed_i_r,
                   speed_d_r, speed_pwm_feedforward_r, speed_pwm_min_r, test_speed);
            diff_speedl_expect = test_speed;
            diff_speedr_expect = test_speed;
            speed_goal_l = (float)test_speed;
            speed_goal_r = (float)test_speed;
            motor_pid_left();
            motor_pid_right();
            return;
        }

        if (!started)
        {
            reset_speed_pid_state();
            speed_tune_stats_reset(&stats_l);
            speed_tune_stats_reset(&stats_r);
            mode5_best_score_l = 999999.0f;
            mode5_best_score_r = 999999.0f;
            candidate_index = 0;
            target_index = 0;
            settle_tick = 0;
            sample_tick = 0;
            candidate_score = 0.0f;
            printed_header = 0;
            started = 1;
            mode5_auto_done = 0;
            printf("[MODE5][SCAN] start grid scan: candidates=%d speeds=80/120/160/200/240/200/160/120. auto stop.\r\n",
                   candidate_count);
        }

        if (mode5_auto_done)
        {
            motor1_pwm_1.set_duty(0);
            motor2_pwm_2.set_duty(0);
            return;
        }

        if (candidate_index >= candidate_count)
        {
            motor1_pwm_1.set_duty(0);
            motor2_pwm_2.set_duty(0);
            mode5_auto_done = 1;
            extern volatile int car_start_flag;
            car_start_flag = 0;
            printf("[MODE5][BEST] L kp=%.3f ki=%.4f kd=%.3f ff=%.1f min=%d | R kp=%.3f ki=%.4f kd=%.3f ff=%.1f min=%d "
                   "score=%.1f\r\n",
                   mode5_best_kp_l, mode5_best_ki_l, mode5_best_kd_l, mode5_best_ff_l, mode5_best_min_l,
                   mode5_best_kp_r, mode5_best_ki_r, mode5_best_kd_r, mode5_best_ff_r, mode5_best_min_r,
                   mode5_best_score_l);
            printf("[MODE5][DONE] motor stopped.\r\n");
            return;
        }

        const Mode5Candidate &c = candidates[candidate_index];
        speed_p_l = c.kp_l;
        speed_i_l = c.ki_l;
        speed_d_l = c.kd_l;
        speed_pwm_feedforward_l = c.ff_l;
        speed_pwm_min_l = c.min_l;
        speed_p_r = c.kp_r;
        speed_i_r = c.ki_r;
        speed_d_r = c.kd_r;
        speed_pwm_feedforward_r = c.ff_r;
        speed_pwm_min_r = c.min_r;

        int target_speed = target_table[target_index];
        diff_speedl_expect = target_speed;
        diff_speedr_expect = target_speed;
        speed_goal_l = (float)target_speed;
        speed_goal_r = (float)target_speed;

        if (!printed_header)
        {
            printed_header = 1;
            printf("[MODE5][SCAN] cand=%d/%d L %.3f %.4f %.3f ff=%.1f min=%d | R %.3f %.4f %.3f ff=%.1f min=%d\r\n",
                   candidate_index + 1, candidate_count, c.kp_l, c.ki_l, c.kd_l, c.ff_l, c.min_l, c.kp_r, c.ki_r,
                   c.kd_r, c.ff_r, c.min_r);
        }

        motor_pid_left();
        motor_pid_right();

        if (settle_tick < 30)
        {
            settle_tick++;
            return;
        }

        speed_tune_stats_update(&stats_l, (float)encoderA_count, speed_error_l);
        speed_tune_stats_update(&stats_r, (float)encoderB_count, speed_error_r);
        sample_tick++;

        if (sample_tick >= 50)
        {
            float avg_speed_l = stats_l.sum_speed / (float)stats_l.samples;
            float avg_speed_r = stats_r.sum_speed / (float)stats_r.samples;
            float avg_abs_error_l = stats_l.sum_abs_error / (float)stats_l.samples;
            float avg_abs_error_r = stats_r.sum_abs_error / (float)stats_r.samples;
            float abs_target = abs_float((float)target_speed);
            float overshoot_l = (stats_l.max_speed - abs_target) / (abs_target + 1.0f);
            float overshoot_r = (stats_r.max_speed - abs_target) / (abs_target + 1.0f);
            if (overshoot_l < 0.0f)
                overshoot_l = 0.0f;
            if (overshoot_r < 0.0f)
                overshoot_r = 0.0f;
            float stop_penalty_l = (avg_speed_l < abs_target * 0.35f) ? 120.0f : 0.0f;
            float stop_penalty_r = (avg_speed_r < abs_target * 0.35f) ? 120.0f : 0.0f;
            float score = avg_abs_error_l + avg_abs_error_r + (overshoot_l + overshoot_r) * 55.0f +
                          (float)(stats_l.sign_changes + stats_r.sign_changes) * 2.0f + stop_penalty_l + stop_penalty_r;
            candidate_score += score;

            printf("[MODE5][SCAN] cand=%d speed=%d L avg=%.1f abs=%.1f max=%.1f sc=%d | R avg=%.1f abs=%.1f max=%.1f "
                   "sc=%d score=%.1f total=%.1f\r\n",
                   candidate_index + 1, target_speed, avg_speed_l, avg_abs_error_l, stats_l.max_speed,
                   stats_l.sign_changes, avg_speed_r, avg_abs_error_r, stats_r.max_speed, stats_r.sign_changes, score,
                   candidate_score);

            target_index++;
            settle_tick = 0;
            sample_tick = 0;
            speed_tune_stats_reset(&stats_l);
            speed_tune_stats_reset(&stats_r);
            reset_speed_pid_state();

            if (target_index >= target_count)
            {
                printf("[MODE5][SCAN] cand=%d total_score=%.1f\r\n", candidate_index + 1, candidate_score);
                if (candidate_score < mode5_best_score_l)
                {
                    mode5_best_score_l = candidate_score;
                    mode5_best_kp_l = c.kp_l;
                    mode5_best_ki_l = c.ki_l;
                    mode5_best_kd_l = c.kd_l;
                    mode5_best_ff_l = c.ff_l;
                    mode5_best_min_l = c.min_l;
                    mode5_best_kp_r = c.kp_r;
                    mode5_best_ki_r = c.ki_r;
                    mode5_best_kd_r = c.kd_r;
                    mode5_best_ff_r = c.ff_r;
                    mode5_best_min_r = c.min_r;
                    printf("[MODE5][SCAN] new best cand=%d score=%.1f\r\n", candidate_index + 1, candidate_score);
                }
                candidate_index++;
                target_index = 0;
                candidate_score = 0.0f;
                printed_header = 0;
            }
        }

        return;
    }

    if (test_mode == 3)
    {
        static int print_cnt = 0;

        motor1_pwm_1.set_duty(0);
        motor2_pwm_2.set_duty(0);

        if (++print_cnt >= 5)
        {
            print_cnt = 0;
            printf("[MODE3][ENC] rawL(e2)=%d rawR(-e1)=%d | mapL=%d mapR=%d\r\n", encoder_raw_l, encoder_raw_r,
                   encoderA_count, encoderB_count);
        }
        return;
    }

    // ════════════════════════════════════════════════
    // 模式6：手动PID调速。串口输入参数，KEY1加速，KEY2减速。
    // ════════════════════════════════════════════════
    if (test_mode == 6)
    {
        diff_speedl_expect = (int16)mode6_target_speed;
        diff_speedr_expect = (int16)mode6_target_speed;
        speed_goal_l = (float)diff_speedl_expect;
        speed_goal_r = (float)diff_speedr_expect;
        motor_pid_left();
        motor_pid_right();

        if (++mode6_print_cnt >= 5)
        {
            mode6_print_cnt = 0;
            printf("[MODE6] target=%d L=%d err=%.1f pwm=%.1f | R=%d err=%.1f pwm=%.1f\r\n", mode6_target_speed,
                   encoderA_count, speed_error_l, speed_pid_out_l, encoderB_count, speed_error_r, speed_pid_out_r);
        }
        return;
    }

    // ════════════════════════════════════════════════
    // 模式0：完整寻迹（正常使用）
    // ════════════════════════════════════════════════
    motor_diff_pid1(); // 差速计算
    speed_goal_l = (float)diff_speedl_expect;
    speed_goal_r = (float)diff_speedr_expect;
    motor_pid_left();  // 左轮PID
    motor_pid_right(); // 右轮PID
}

void motor_init()
{
    motor1_pwm_1.get_dev_info(&motor1_pwm_info);
    motor2_pwm_2.get_dev_info(&motor2_pwm_info);
}

void encoder_test()
{
    printf("L:%d | R:%d\r\n", encoderA_count, encoderB_count);
}

// 临时诊断函数：打印每行中线位置（调试完后删除！）
void debug_print_centers()
{
}

static int calc_uphill_pwm_boost(float goal, float actual, int *low_speed_count)
{
    float abs_goal = abs_float(goal);
    float abs_actual = abs_float(actual);

    // 只在直线/小偏差时允许爬坡补偿；一进弯就立刻清掉，避免下坡进弯继续加力。
    if (!uphill_boost_allowed || abs_goal < 80.0f)
    {
        *low_speed_count = 0;
        return 0;
    }

    bool speed_too_low = abs_actual < abs_goal * 0.55f;
    bool same_direction = (goal * actual) >= 0.0f || abs_actual < 5.0f;

    if (speed_too_low && same_direction)
    {
        if (*low_speed_count < 30)
            (*low_speed_count)++;
    }
    else
    {
        *low_speed_count = 0;
        return 0;
    }

    if (*low_speed_count < 3)
        return 0;

    int boost = 200 + (*low_speed_count - 3) * 20;
    if (boost > 650)
        boost = 650;

    return goal > 0.0f ? boost : -boost;
}

void motor_pid_left()
{
    static int low_speed_count_l = 0;

    // encoderA_count 已是本周期增量（pulse/周期）
    float actual_speed = (float)encoderA_count; // 当前速度估计

    speed_error_l = speed_goal_l - actual_speed; // err = goal - actual

    // P 项
    float pout = speed_p_l * speed_error_l;

    // I 项（关键：限幅要小！）
    speed_integral_l += speed_i_l * speed_error_l;
    if (speed_integral_l > 350.0f)
        speed_integral_l = 350.0f;
    if (speed_integral_l < -350.0f)
        speed_integral_l = -350.0f;

    // D 项（加滤波防噪声）
    float dout_raw = speed_d_l * (speed_error_l - speed_last_error_l);
    speed_dout_filtered_l = 0.8f * speed_dout_filtered_l + 0.2f * dout_raw;
    speed_last_error_l = speed_error_l;

    // 总输出
    speed_pid_out_l = pout + speed_integral_l + speed_dout_filtered_l;

    // 限幅（PWM范围）
    if (speed_pid_out_l > pwm_max)
        speed_pid_out_l = pwm_max;
    if (speed_pid_out_l < pwm_min)
        speed_pid_out_l = pwm_min;

    // 左轮输出
    int final_pwm_l = (int)(speed_goal_l * speed_pwm_feedforward_l + speed_pid_out_l);
    final_pwm_l += calc_uphill_pwm_boost(speed_goal_l, actual_speed, &low_speed_count_l);
    if (final_pwm_l > pwm_max)
        final_pwm_l = pwm_max;
    if (final_pwm_l < pwm_min)
        final_pwm_l = pwm_min;
    if (speed_goal_l == 0.0f)
        final_pwm_l = 0;
    else if (speed_goal_l > 0.0f)
    {
        if (final_pwm_l < 0)
            final_pwm_l = 0;
        else if (final_pwm_l > 0 && final_pwm_l < speed_pwm_min_l)
            final_pwm_l = speed_pwm_min_l;
    }
    else
    {
        if (final_pwm_l > 0)
            final_pwm_l = 0;
        else if (final_pwm_l < 0 && final_pwm_l > -speed_pwm_min_l)
            final_pwm_l = -speed_pwm_min_l;
    }

    if (final_pwm_l >= 0)
    {
        motor1_gpio.set_level(0);
        motor1_pwm_1.set_duty(final_pwm_l);
    }
    else
    {
        motor1_gpio.set_level(1);
        motor1_pwm_1.set_duty(-final_pwm_l);
    }
}

void motor_pid_right()
{
    static int low_speed_count_r = 0;

    // encoderB_count 已是本周期增量（pulse/周期）
    float actual_speed = (float)encoderB_count; // 当前速度估计

    speed_error_r = speed_goal_r - actual_speed; // err = goal - actual

    // P 项
    float pout = speed_p_r * speed_error_r;

    // I 项（关键：限幅要小！）
    speed_integral_r += speed_i_r * speed_error_r;
    if (speed_integral_r > 350.0f)
        speed_integral_r = 350.0f;
    if (speed_integral_r < -350.0f)
        speed_integral_r = -350.0f;

    // D 项（加滤波防噪声）
    float dout_raw = speed_d_r * (speed_error_r - speed_last_error_r);
    speed_dout_filtered_r = 0.8f * speed_dout_filtered_r + 0.2f * dout_raw;
    speed_last_error_r = speed_error_r;

    // 总输出
    speed_pid_out_r = pout + speed_integral_r + speed_dout_filtered_r;

    // 限幅（PWM范围）
    if (speed_pid_out_r > pwm_max)
        speed_pid_out_r = pwm_max;
    if (speed_pid_out_r < pwm_min)
        speed_pid_out_r = pwm_min;

    // 右轮输出
    int final_pwm_r = (int)(speed_goal_r * speed_pwm_feedforward_r + speed_pid_out_r);
    final_pwm_r += calc_uphill_pwm_boost(speed_goal_r, actual_speed, &low_speed_count_r);
    if (final_pwm_r > pwm_max)
        final_pwm_r = pwm_max;
    if (final_pwm_r < pwm_min)
        final_pwm_r = pwm_min;
    if (speed_goal_r == 0.0f)
        final_pwm_r = 0;
    else if (speed_goal_r > 0.0f)
    {
        if (final_pwm_r < 0)
            final_pwm_r = 0;
        else if (final_pwm_r > 0 && final_pwm_r < speed_pwm_min_r)
            final_pwm_r = speed_pwm_min_r;
    }
    else
    {
        if (final_pwm_r > 0)
            final_pwm_r = 0;
        else if (final_pwm_r < 0 && final_pwm_r > -speed_pwm_min_r)
            final_pwm_r = -speed_pwm_min_r;
    }

    if (final_pwm_r < 0)
    {
        motor2_gpio.set_level(1);
        motor2_pwm_2.set_duty(-final_pwm_r);
    }
    else
    {
        motor2_gpio.set_level(0);
        motor2_pwm_2.set_duty(final_pwm_r);
    }
}

void motor_diff_pid1()
{
    static float last_turn_error = 0;
    static float filtered_turn_output = 0;

    // 图像偏差（根据实际调整中线值）
    float target_center = 38.0f + avoid_get_trans_line();
    target_center = clamp_float(target_center, 5.0f, 75.0f);
    float turn_error = target_center - ImageStatus.Det_True;
    // 死区控制
    if (turn_error > -1.5f && turn_error < 1.5f)
    {
        turn_error = 0;
    }

    float abs_turn_error = abs_float(turn_error);
    bool ring_turning = (ImageFlag.image_element_rings_flag >= 1 && ImageFlag.image_element_rings_flag <= 8);
    bool ring_detected = (ImageStatus.Road_type == LeftCirque || ImageStatus.Road_type == RightCirque ||
                          ImageFlag.image_element_rings_flag != 0);
    uphill_boost_allowed = (!ring_detected && abs_turn_error <= 3.0f);
    float current_kp = diff_kp;
    float current_kd = 0.20f;
    if (abs_turn_error <= 3.5f)
    {
        current_kp = turn_kp_low;
        current_kd = 0.12f;
    }
    else if (!ring_turning && abs_turn_error >= 12.0f)
    {
        current_kp = turn_kp_sharp; // strongest normal bend turn
        current_kd = 0.34f;
    }
    else if (abs_turn_error >= 8.0f)
    {
        current_kp = turn_kp_big; // stronger bend turn
        current_kd = 0.30f;
    }
    else
    {
        current_kp = turn_kp_mid;
        current_kd = 0.22f;
    }

    // 转向 PD 控制

    float turn_output = current_kp * turn_error + current_kd * (turn_error - last_turn_error);
    if (ring_turning)
        turn_output *= turn_ring_multiplier;
    bool turn_cross_zero = (turn_error * last_turn_error) < 0.0f;
    last_turn_error = turn_error;

    // 转向限幅：允许急弯接近外侧正转、内侧反转，但避免D项尖峰过猛
    float turn_limit = 650.0f;
    if (turn_output > turn_limit)
        turn_output = turn_limit;
    if (turn_output < -turn_limit)
        turn_output = -turn_limit;

    // 基础速度（慢速模式）
    if (turn_cross_zero)
    {
        filtered_turn_output = 0.0f;
    }

    float max_turn_step = (abs_turn_error >= 8.0f) ? 320.0f : 180.0f;
    float turn_delta = turn_output - filtered_turn_output;
    if (turn_delta > max_turn_step)
        turn_delta = max_turn_step;
    if (turn_delta < -max_turn_step)
        turn_delta = -max_turn_step;
    filtered_turn_output += turn_delta;
    filtered_turn_output = 0.35f * filtered_turn_output + 0.65f * turn_output;
    if (abs_turn_error <= 3.0f)
    {
        filtered_turn_output *= 0.25f;
    }

    int current_base_speed = line_base_speed;

    // 环岛限速：只要进入环岛相关状态，就先按 Ring 倍率限制最高速度。

    // Ring/Small/Mid/Big/Sharp 都可以在 Speed 菜单里以 0.1 步长调整。
    int ring_speed_limit = (int)(line_base_speed * speed_ratio_ring);
    if (ring_detected && current_base_speed > ring_speed_limit)
        current_base_speed = ring_speed_limit;

    // 初见环岛阶段额外减速，给入环识别和补线留反应时间。
    if (ImageFlag.image_element_rings_flag == 1)
        current_base_speed -= 30;

    // 普通弯道分档限速：保留一定速度，但避免入弯外轮猛加速导致转向半径过大。
    // 小弯/中弯通常可保持全速；大弯/急弯按菜单倍率限制。
    if (abs_turn_error >= 16.0f)
    {
        int speed_limit = (int)(line_base_speed * speed_ratio_sharp);
        if (current_base_speed > speed_limit)
            current_base_speed = speed_limit;
    }
    else if (abs_turn_error >= 12.0f)
    {
        int speed_limit = (int)(line_base_speed * speed_ratio_big);
        if (current_base_speed > speed_limit)
            current_base_speed = speed_limit;
    }
    else if (abs_turn_error >= 8.0f)
    {
        int speed_limit = (int)(line_base_speed * speed_ratio_mid);
        if (current_base_speed > speed_limit)
            current_base_speed = speed_limit;
    }
    else if (abs_turn_error >= 4.5f)
    {
        int speed_limit = (int)(line_base_speed * speed_ratio_small);
        if (current_base_speed > speed_limit)
            current_base_speed = speed_limit;
    }

    // 最低速度保护：防止限速过低导致电机克服不了静摩擦。
    if (current_base_speed < 45)
        current_base_speed = 45;

    // 计算左右轮目标速度。
    // 弯道不要再用“外轮大幅加速 + 内轮减速”的对称差速，否则基础速度低时，
    // 一进弯外轮仍会被推到很高，表现为弯道猛加速、转向半径变大。
    int turn_cmd = (int)filtered_turn_output;
    // 高速巡线折中：外轮适当加速保持过弯速度，内轮减速形成转向力矩。
    // 大弯仍保留更多内轮减速，避免只提速不转向。
    float outer_gain = (abs_turn_error >= 12.0f) ? 0.50f : 0.65f;
    float inner_gain = (abs_turn_error >= 12.0f) ? 1.65f : 1.45f;

    if (turn_cmd >= 0)
    {
        diff_speedl_expect = current_base_speed + (int)(turn_cmd * outer_gain);
        diff_speedr_expect = current_base_speed - (int)(turn_cmd * inner_gain);
    }
    else
    {
        int turn_abs = -turn_cmd;
        diff_speedl_expect = current_base_speed - (int)(turn_abs * inner_gain);
        diff_speedr_expect = current_base_speed + (int)(turn_abs * outer_gain);
    }

    // 极限保护：左右轮目标速度统一限制在 -Limit 到 +Limit。
    if (diff_speedl_expect < -speed_expect_limit)
        diff_speedl_expect = -speed_expect_limit;
    if (diff_speedr_expect < -speed_expect_limit)
        diff_speedr_expect = -speed_expect_limit;

    if (diff_speedl_expect > speed_expect_limit)
        diff_speedl_expect = speed_expect_limit;
    if (diff_speedr_expect > speed_expect_limit)
        diff_speedr_expect = speed_expect_limit;
}
