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

// 测试模式：0=寻迹, 1=速度测试, 2=固定PWM, 3=编码器观察, 4=映射验证
int test_mode = 0;
int mode2_pwm = 1200;

int pwm_max = 3500, pwm_min = -3500; // PWM限幅（duty=800对应8%占空比）

int16 speed_begin = 500;
int16 speed_expect = 0;

float diff_speed_error;
int16 diff_speedl_expect, diff_speedr_expect;

float diff_kp = 2.6f, diff_kd = 3.5f; // 差速PID参数（快速响应）
float k = 2.868;
float dif = 0;

float k1 = 0.6;
float y, x;

// 测试参数
int test_wheel = 1;  // 测试轮选择
int test_speed = 60; // 测试目标速度

int speed_pwm_min_l = 750;
int speed_pwm_min_r = 725;
float speed_pwm_feedforward_l = 10.6f;
float speed_pwm_feedforward_r = 10.3f;
static int line_base_speed = 200;

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
    if (speed > 260)
        speed = 260;
    line_base_speed = speed;
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

    // ✅ 低速PID参数（Kp=8.0确保pwm能达到800启动）
    speed_p_l = 0.35f;
    speed_i_l = 0.020f;
    speed_d_l = 0.0f;

    speed_p_r = 0.35f;
    speed_i_r = 0.020f;
    speed_d_r = 0.0f;
}

// 临时诊断函数声明（调试完后删除！）
void debug_print_centers();

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

    encoder_dir_2.clear_count(); // 清零！
    encoder_dir_1.clear_count(); // 清零！

    // ✅ 关键：同步差速环目标到PID目标（仅在寻迹模式）
    int16 encoder_swap_temp = encoderA_count;
    encoderA_count = encoderB_count;
    encoderB_count = encoder_swap_temp;

    if (test_mode == 0)
    {
        speed_goal_l = (float)diff_speedl_expect;
        speed_goal_r = (float)diff_speedr_expect;
    }

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
        static int started = 0;
        static int cnt = 0;
        static int sample_count = 0;
        static int stable_windows = 0;
        static int tune_locked = 0;
        static int sign_changes_l = 0;
        static int sign_changes_r = 0;
        static float last_error_l = 0.0f;
        static float last_error_r = 0.0f;
        static float sum_error_l = 0.0f;
        static float sum_error_r = 0.0f;
        static float sum_abs_error_l = 0.0f;
        static float sum_abs_error_r = 0.0f;
        static float max_abs_error_l = 0.0f;
        static float max_abs_error_r = 0.0f;

        if (!started)
        {
            reset_speed_pid_state();
            diff_speedl_expect = test_speed;
            diff_speedr_expect = test_speed;
            speed_goal_l = (float)diff_speedl_expect;
            speed_goal_r = (float)diff_speedr_expect;

            speed_p_l = 0.9f;
            speed_i_l = 0.02f;
            speed_d_l = 0.08f;
            speed_p_r = 0.9f;
            speed_i_r = 0.02f;
            speed_d_r = 0.08f;

            cnt = 0;
            sample_count = 0;
            stable_windows = 0;
            tune_locked = 0;
            sign_changes_l = 0;
            sign_changes_r = 0;
            last_error_l = 0.0f;
            last_error_r = 0.0f;
            sum_error_l = 0.0f;
            sum_error_r = 0.0f;
            sum_abs_error_l = 0.0f;
            sum_abs_error_r = 0.0f;
            max_abs_error_l = 0.0f;
            max_abs_error_r = 0.0f;
            started = 1;
        }

        motor_pid_left();
        motor_pid_right();

        float abs_error_l = abs_float(speed_error_l);
        float abs_error_r = abs_float(speed_error_r);

        if (sample_count > 0 &&
            ((speed_error_l > 0.0f && last_error_l < 0.0f) || (speed_error_l < 0.0f && last_error_l > 0.0f)))
        {
            sign_changes_l++;
        }
        if (sample_count > 0 &&
            ((speed_error_r > 0.0f && last_error_r < 0.0f) || (speed_error_r < 0.0f && last_error_r > 0.0f)))
        {
            sign_changes_r++;
        }

        last_error_l = speed_error_l;
        last_error_r = speed_error_r;
        sum_error_l += speed_error_l;
        sum_error_r += speed_error_r;
        sum_abs_error_l += abs_error_l;
        sum_abs_error_r += abs_error_r;
        if (abs_error_l > max_abs_error_l)
            max_abs_error_l = abs_error_l;
        if (abs_error_r > max_abs_error_r)
            max_abs_error_r = abs_error_r;

        sample_count++;

        if (++cnt % 25 == 0)
        {
            float avg_error_l = sum_error_l / (float)sample_count;
            float avg_error_r = sum_error_r / (float)sample_count;
            float avg_abs_error_l = sum_abs_error_l / (float)sample_count;
            float avg_abs_error_r = sum_abs_error_r / (float)sample_count;

            int stable_l = (abs_float(avg_error_l) < 8.0f && avg_abs_error_l < 13.0f && max_abs_error_l < 85.0f);
            int stable_r = (abs_float(avg_error_r) < 8.0f && avg_abs_error_r < 13.0f && max_abs_error_r < 85.0f);

            if (!tune_locked && stable_l && stable_r)
                stable_windows++;
            else if (!tune_locked && stable_windows > 0)
                stable_windows--;

            if (!tune_locked && stable_windows >= 3)
            {
                tune_locked = 1;
                reset_speed_pid_state();
            }

            if (tune_locked && avg_error_l > 6.0f)
            {
                speed_pwm_min_l += 10;
                speed_pwm_feedforward_l = clamp_float(speed_pwm_feedforward_l + 0.1f, 4.0f, 24.0f);
                reset_speed_pid_state();
            }
            else if (tune_locked && avg_error_l < -6.0f)
            {
                speed_pwm_min_l -= 10;
                speed_pwm_feedforward_l = clamp_float(speed_pwm_feedforward_l - 0.1f, 4.0f, 24.0f);
                reset_speed_pid_state();
            }

            if (tune_locked && avg_error_r > 6.0f)
            {
                speed_pwm_min_r += 10;
                speed_pwm_feedforward_r = clamp_float(speed_pwm_feedforward_r + 0.1f, 4.0f, 24.0f);
                reset_speed_pid_state();
            }
            else if (tune_locked && avg_error_r < -6.0f)
            {
                speed_pwm_min_r -= 10;
                speed_pwm_feedforward_r = clamp_float(speed_pwm_feedforward_r - 0.1f, 4.0f, 24.0f);
                reset_speed_pid_state();
            }

            if (!tune_locked && avg_error_l > 6.0f)
            {
                speed_pwm_min_l += 25;
                speed_pwm_feedforward_l = clamp_float(speed_pwm_feedforward_l + 0.3f, 4.0f, 24.0f);
            }
            else if (!tune_locked && avg_error_l < -6.0f)
            {
                speed_pwm_min_l -= 25;
                speed_pwm_feedforward_l = clamp_float(speed_pwm_feedforward_l - 0.3f, 4.0f, 24.0f);
            }

            if (!tune_locked && avg_error_r > 6.0f)
            {
                speed_pwm_min_r += 25;
                speed_pwm_feedforward_r = clamp_float(speed_pwm_feedforward_r + 0.3f, 4.0f, 24.0f);
            }
            else if (!tune_locked && avg_error_r < -6.0f)
            {
                speed_pwm_min_r -= 25;
                speed_pwm_feedforward_r = clamp_float(speed_pwm_feedforward_r - 0.3f, 4.0f, 24.0f);
            }

            speed_pwm_min_l = (int)clamp_float((float)speed_pwm_min_l, 450.0f, 1300.0f);
            speed_pwm_min_r = (int)clamp_float((float)speed_pwm_min_r, 450.0f, 1300.0f);

            if (!tune_locked && (sign_changes_l > 6 || max_abs_error_l > 80.0f))
            {
                speed_p_l = clamp_float(speed_p_l * 0.9f, 0.35f, 4.0f);
                speed_d_l = clamp_float(speed_d_l * 0.85f, 0.0f, 1.2f);
            }
            else if (!tune_locked && avg_abs_error_l > 12.0f && sign_changes_l <= 3)
            {
                speed_p_l = clamp_float(speed_p_l + 0.08f, 0.35f, 4.0f);
                speed_d_l = clamp_float(speed_d_l + 0.02f, 0.0f, 1.2f);
            }

            if (!tune_locked && (sign_changes_r > 6 || max_abs_error_r > 80.0f))
            {
                speed_p_r = clamp_float(speed_p_r * 0.9f, 0.35f, 4.0f);
                speed_d_r = clamp_float(speed_d_r * 0.85f, 0.0f, 1.2f);
            }
            else if (!tune_locked && avg_abs_error_r > 12.0f && sign_changes_r <= 3)
            {
                speed_p_r = clamp_float(speed_p_r + 0.08f, 0.35f, 4.0f);
                speed_d_r = clamp_float(speed_d_r + 0.02f, 0.0f, 1.2f);
            }

            sample_count = 0;
            sign_changes_l = 0;
            sign_changes_r = 0;
            sum_error_l = 0.0f;
            sum_error_r = 0.0f;
            sum_abs_error_l = 0.0f;
            sum_abs_error_r = 0.0f;
            max_abs_error_l = 0.0f;
            max_abs_error_r = 0.0f;
        }

        return;
    }

    if (test_mode == 3)
    {
        static int started = 0;

        if (!started)
        {
            reset_speed_pid_state();
            started = 1;

            // 在这里调整PID参数
            speed_p_l = 1.2f;
            speed_i_l = 0.05f;
            speed_d_l = 0.8f;
            speed_p_r = 1.2f;
            speed_i_r = 0.05f;
            speed_d_r = 0.8f;

            setup_speed_test_target();
        }

        motor_pid_left();
        motor_pid_right();
        return;
    }

    // ════════════════════════════════════════════════
    // 模式0：完整寻迹（正常使用）
    // ════════════════════════════════════════════════
    // 绕行接管
    if (avoid_control_left() || avoid_control_right())
    {
        speed_goal_l = (float)diff_speedl_expect;
        speed_goal_r = (float)diff_speedr_expect;
        motor_pid_left();
        motor_pid_right();
        return;
    }

    motor_diff_pid1(); // 差速计算
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

void motor_pid_left()
{
    // encoderA_count 已是本周期增量（pulse/周期）
    float actual_speed = (float)encoderA_count; // 当前速度估计

    speed_error_l = speed_goal_l - actual_speed; // err = goal - actual

    // P 项
    float pout = speed_p_l * speed_error_l;

    // I 项（关键：限幅要小！）
    speed_integral_l += speed_i_l * speed_error_l;
    if (speed_integral_l > 50.0f)
        speed_integral_l = 50.0f;
    if (speed_integral_l < -50.0f)
        speed_integral_l = -50.0f;

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
    // encoderB_count 已是本周期增量（pulse/周期）
    float actual_speed = (float)encoderB_count; // 当前速度估计

    speed_error_r = speed_goal_r - actual_speed; // err = goal - actual

    // P 项
    float pout = speed_p_r * speed_error_r;

    // I 项（关键：限幅要小！）
    speed_integral_r += speed_i_r * speed_error_r;
    if (speed_integral_r > 50.0f)
        speed_integral_r = 50.0f;
    if (speed_integral_r < -50.0f)
        speed_integral_r = -50.0f;

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
    float turn_error = 40 - ImageStatus.Det_True;
    // 死区控制
    if (turn_error > -1.5f && turn_error < 1.5f)
    {
        turn_error = 0;
    }

    float abs_turn_error = abs_float(turn_error);
    float current_kp = diff_kp;
    float current_kd = 0.20f;
    if (abs_turn_error <= 3.5f)
    {
        current_kp = diff_kp * 0.7f;
        current_kd = 0.12f;
    }
    else if (abs_turn_error >= 8.0f)
    {
        current_kp = 9.0f; // stronger bend turn
        current_kd = 0.30f;
    }
    else
    {
        current_kp = 5.0f;
        current_kd = 0.22f;
    }

    // 转向 PD 控制

    float turn_output = current_kp * turn_error + current_kd * (turn_error - last_turn_error);
    bool turn_cross_zero = (turn_error * last_turn_error) < 0.0f;
    last_turn_error = turn_error;

    // 转向限幅：允许急弯接近外侧正转、内侧反转，但避免D项尖峰过猛
    float turn_limit = 520.0f;
    if (turn_output > turn_limit)
        turn_output = turn_limit;
    if (turn_output < -turn_limit)
        turn_output = -turn_limit;

    // 基础速度（慢速模式）
    if (turn_cross_zero)
    {
        filtered_turn_output = 0.0f;
    }

    float max_turn_step = (abs_turn_error >= 8.0f) ? 220.0f : 70.0f;
    float turn_delta = turn_output - filtered_turn_output;
    if (turn_delta > max_turn_step)
        turn_delta = max_turn_step;
    if (turn_delta < -max_turn_step)
        turn_delta = -max_turn_step;
    filtered_turn_output += turn_delta;
    filtered_turn_output = 0.65f * filtered_turn_output + 0.35f * turn_output;
    if (abs_turn_error <= 3.0f)
    {
        filtered_turn_output *= 0.25f;
    }

    int current_base_speed = line_base_speed;
    bool ring_detected = (ImageStatus.Road_type == LeftCirque ||
                          ImageStatus.Road_type == RightCirque ||
                          ImageFlag.image_element_rings_flag != 0);
    if (ring_detected && current_base_speed > 150)
        current_base_speed = 150;
    if (abs_turn_error > 3.0f)
        current_base_speed -= (int)((abs_turn_error - 3.0f) * 9.0f);
    if (abs_turn_error >= 12.0f && current_base_speed > 60)
        current_base_speed = 60;
    else if (abs_turn_error >= 8.0f && current_base_speed > 90)
        current_base_speed = 90;
    else if (abs_turn_error >= 4.5f && current_base_speed > 140)
        current_base_speed = 140;
    if (current_base_speed < 45)
        current_base_speed = 45;

    // 计算左右轮目标速度
    diff_speedl_expect = current_base_speed + (int)filtered_turn_output;
    diff_speedr_expect = current_base_speed - (int)filtered_turn_output;

    // 极限保护（慢速模式）
    if (diff_speedl_expect < -260)
        diff_speedl_expect = -260;
    if (diff_speedr_expect < -260)
        diff_speedr_expect = -260;

    if (diff_speedl_expect > 260)
        diff_speedl_expect = 260;
    if (diff_speedr_expect > 260)
        diff_speedr_expect = 260;
}
