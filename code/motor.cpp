#include "motor.hpp"
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
int test_mode = 2;
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

void motor_argument()
{
    // ✅ 慢速目标：100 pulse/周期（对应duty≈800，8%占空比）
    speed_goal_l = 120.0f;
    speed_goal_r = 120.0f;

    diff_speedl_expect = (int16)speed_goal_l;
    diff_speedr_expect = (int16)speed_goal_r;

    // ✅ 低速PID参数（Kp=8.0确保pwm能达到800启动）
    speed_p_l = 20.5f; // 提高！err=100→pwm=800
    speed_i_l = 0.0f;  // 小积分防饱和
    speed_d_l = 0.0f;  // 降低D项

    speed_p_r = 20.0f;
    speed_i_r = 0.0f;
    speed_d_r = 0.0f;
}

// 临时诊断函数声明（调试完后删除！）
void debug_print_centers();

void motor_control()
{
    // ════════════════════════════════════════════════
    // 1. 获取编码器数据（速度闭环：读完后清零！）
    // ════════════════════════════════════════════════
    encoderA_count = encoder_dir_2.get_count();  // 左轮Encoder2
    encoderB_count = -encoder_dir_1.get_count(); // 右轮Encoder1（取反）

    encoder_dir_2.clear_count(); // 清零！
    encoder_dir_1.clear_count(); // 清零！

    // ✅ 关键：同步差速环目标到PID目标（仅在寻迹模式）
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
    if (test_mode == 3)
    {
        static int started = 0;
        static int cnt = 0;

        if (!started)
        {
            speed_pid_out_l = 20;
            speed_pid_out_r = 20;
            started = 1;

            // 在这里调整PID参数
            speed_p_l = 1.2f;
            speed_i_l = 0.05f;
            speed_d_l = 0.8f;
            speed_p_r = 1.2f;
            speed_i_r = 0.05f;
            speed_d_r = 0.8f;

            int test_speed = 60;

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
        }

        motor_pid_left();
        motor_pid_right();

        if (++cnt % 10 == 0)
        {
            printf("L:%d/%d | R:%d/%d\r\n", diff_speedl_expect, encoderA_count, diff_speedr_expect, encoderB_count);
        }
        return;
    }

    // ════════════════════════════════════════════════
    // 模式0：完整寻迹（正常使用）
    // ════════════════════════════════════════════════

    motor_diff_pid1(); // 差速计算

    motor_pid_left();  // 左轮PID
    motor_pid_right(); // 右轮PID

    debug_print_centers(); // 调试用中线打印

    // 简单的寻迹诊断信息（速度闭环格式）
    static int cnt0 = 0;
    if (++cnt0 % 50 == 0)
    {
        // 计算最终PWM（和输出逻辑一致）
        int final_pwm_l = speed_pid_out_l;
        if (final_pwm_l > pwm_max)
            final_pwm_l = pwm_max;
        if (final_pwm_l < pwm_min)
            final_pwm_l = pwm_min;

        int final_pwm_r = speed_pid_out_r;
        if (final_pwm_r > pwm_max)
            final_pwm_r = pwm_max;
        if (final_pwm_r < pwm_min)
            final_pwm_r = pwm_min;

        printf("[TRACK] Det=%d | L:goal=%d act=%d err=%d pwm=%d | R:goal=%d act=%d err=%d pwm=%d\r\n",
               ImageStatus.Det_True, diff_speedl_expect, encoderA_count, diff_speedl_expect - encoderA_count,
               final_pwm_l, diff_speedr_expect, encoderB_count, diff_speedr_expect - encoderB_count, final_pwm_r);
    }
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
    static int cnt = 0;
    if (++cnt % 30 == 0)
    { // 每30帧打印一次
        printf("[CENTERS] ");
        for (int y = 0; y < LCDH; y += 5)
        { // 每5行打印一次
            printf("%d ", ImageDeal[y].Center);
        }
        printf("| Det_True=%d TowPoint=%d\r\n", ImageStatus.Det_True, ImageStatus.TowPoint_True);
    }
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
    int final_pwm_l = (int)speed_pid_out_l;
    if (final_pwm_l > pwm_max)
        final_pwm_l = pwm_max;
    if (final_pwm_l < pwm_min)
        final_pwm_l = pwm_min;
    if (final_pwm_l > 0 && final_pwm_l < 1200)
        final_pwm_l = 1200; // 正向最小 600
    if (final_pwm_l < 0 && final_pwm_l > -1200)
        final_pwm_l = -1200; // 反向最小 -600

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
    int final_pwm_r = (int)speed_pid_out_r;
    if (final_pwm_r > pwm_max)
        final_pwm_r = pwm_max;
    if (final_pwm_r < pwm_min)
        final_pwm_r = pwm_min;
    if (final_pwm_r > 0 && final_pwm_r < 1200)
        final_pwm_r = 1200; // 正向最小 600
    if (final_pwm_r < 0 && final_pwm_r > -1200)
        final_pwm_r = -1200; // 反向最小 -600

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

    // 图像偏差（根据实际调整中线值）
    float turn_error = 40 - ImageStatus.Det_True;
    // 死区控制
    if (turn_error > -4.0f && turn_error < 4.0f)
    {
        turn_error = 0;
    }

    float current_kp = diff_kp;
    if (turn_error >= -10.0f && turn_error <= 10.0f)
    {
        current_kp = diff_kp * 0.5f; // 温柔一点，防止抖动
    }

    // 转向 PD 控制
    float turn_output = current_kp * turn_error + diff_kd * (turn_error - last_turn_error);
    last_turn_error = turn_error;

    // 转向限幅（小转弯，慢速模式）
    float turn_limit = 55.0f;
    if (turn_output > turn_limit)
        turn_output = turn_limit;
    if (turn_output < -turn_limit)
        turn_output = -turn_limit;

    // 基础速度（慢速模式）
    int current_base_speed = 100 - (int)(abs(turn_error) * 0.3f);
    if (current_base_speed < 40)
        current_base_speed = 40; // 最低40

    // 计算左右轮目标速度
    diff_speedl_expect = current_base_speed + (int)turn_output;
    diff_speedr_expect = current_base_speed - (int)turn_output;

    // 极限保护（慢速模式）
    if (diff_speedl_expect < 0)
        diff_speedl_expect = 0;
    if (diff_speedr_expect < 0)
        diff_speedr_expect = 0;

    if (diff_speedl_expect > 200)
        diff_speedl_expect = 200;
    if (diff_speedr_expect > 200)
        diff_speedr_expect = 200;
}
