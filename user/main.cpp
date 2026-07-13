/*********************************************************************************************************************
 * LS2K0300 Opensourec Library 即（LS2K0300 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是LS2K0300 开源库的一部分
 *
 * LS2K0300 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          main
 * 公司名称          成都逐飞科技有限公司
 * 适用平台          LS2K0300
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者           备注
 * 2025-12-27        大W            first version
 * 2026-01-09        AI助手         融入E08_02 RGB标准 + 正交编码器(QUAD) + 智能车逻辑优化 + 修复所有BUG
 ********************************************************************************************************************/

#include "zf_common_headfile.hpp"
#include "zf_driver_tcp_client.hpp"
#include "zf_device_uvc.hpp"
#include "zf_driver_pwm.hpp"
#include "init.hpp"
#include "image.hpp"
#include "config.hpp"
#include "ips200_draw.hpp"
#include "CarVision.hpp"
#include "avoid.hpp"
#include "beep.hpp"
#include "menu.hpp"
// ====================== 全局宏定义 ======================
#define SERVER_IP "10.18.55.68"

// ====================== 网络配置宏定义 ======================
#define PORT 8086
#define IMAGE_TRANSFER_INTERVAL 3
#define IMAGE_TRANSFER_DEFAULT_STATE IMAGE_TRANSFER_OFF

// #define DEBUG_PRINT



// ======================清理退出函数======================
void sigint_handler(int sig);
void cleanup();

// ====================== 边界信息配置宏定义 ======================
#define INCLUDE_BOUNDARY_TYPE 0
#define BOUNDARY_NUM (UVC_HEIGHT * 4 / 2)

// ====================== 全局数组定义 ======================
uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];
uint8 x1_boundary[UVC_HEIGHT], x2_boundary[UVC_HEIGHT], x3_boundary[UVC_HEIGHT];
uint8 y1_boundary[UVC_WIDTH], y2_boundary[UVC_WIDTH], y3_boundary[UVC_WIDTH];
uint8 image_copy[LCDH][LCDW];

// ====================== TCP通信包装函数 ======================
zf_driver_tcp_client *g_tcp_client = nullptr;

enum image_transfer_state_t
{
    IMAGE_TRANSFER_OFF = 0,
    IMAGE_TRANSFER_INIT,
    IMAGE_TRANSFER_RUNNING,
    IMAGE_TRANSFER_ERROR,
};

static image_transfer_state_t image_transfer_state = IMAGE_TRANSFER_DEFAULT_STATE;
static int image_transfer_fail_count = 0;

static uint32 send_wrapper(const uint8 *buff, uint32 length)
{
    uint32 sent = g_tcp_client ? g_tcp_client->send_data(buff, length) : 0;
    if (image_transfer_state == IMAGE_TRANSFER_RUNNING && sent == 0)
    {
        image_transfer_fail_count++;
        if (image_transfer_fail_count >= 3)
        {
            image_transfer_state = IMAGE_TRANSFER_ERROR;
            printf("[IMAGE] transfer disabled: tcp send failed\r\n");
        }
    }
    else if (sent != 0)
    {
        image_transfer_fail_count = 0;
    }
    return sent;
}
static uint32 read_wrapper(uint8 *buff, uint32 length)
{
    return g_tcp_client ? g_tcp_client->read_data(buff, length) : 0;
}

// **************************** 主函数入口 ****************************
int main()
{
    // ====================== 1. 系统初始化 ======================
    signal(SIGINT, sigint_handler);

    init_all();      // 初始化所有外设（内部已包含 motor_argument！）
    Data_Settings(); // 设置图像和PID参数

#if (1 == INCLUDE_BOUNDARY_TYPE || 2 == INCLUDE_BOUNDARY_TYPE || 4 == INCLUDE_BOUNDARY_TYPE)
    int32 i = 0;
#elif (3 == INCLUDE_BOUNDARY_TYPE)
    int32 i = 0;
    int32 j = 0;
#endif

    // ====================== 2. TCP网络初始化 ======================
    zf_driver_tcp_client tcp_client;

    if (image_transfer_state == IMAGE_TRANSFER_INIT)
    {
        g_tcp_client = &tcp_client;
        if (tcp_client.init(SERVER_IP, PORT) == 0)
        {
            tcp_client.set_retry_param(1, 1);
            image_transfer_state = IMAGE_TRANSFER_RUNNING;
            printf("[IMAGE] transfer running\r\n");
        }
        else
        {
            g_tcp_client = nullptr;
            image_transfer_state = IMAGE_TRANSFER_ERROR;
            printf("[IMAGE] transfer disabled: tcp init failed\r\n");
        }

        seekfree_assistant_interface_init(send_wrapper, read_wrapper);
    }
    else
    {
        g_tcp_client = nullptr;
        printf("[IMAGE] transfer off\r\n");
    }

    // ====================== 3. 边界信息配置 ======================
#if (0 == INCLUDE_BOUNDARY_TYPE)
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], LCDW, LCDH);

#elif (1 == INCLUDE_BOUNDARY_TYPE)
    for (i = 0; i < UVC_HEIGHT; i++)
    {
        x1_boundary[i] = 50 - (50 - 20) * i / UVC_HEIGHT;
        x2_boundary[i] = UVC_WIDTH / 2;
        x3_boundary[i] = 70 + (148 - 70) * i / UVC_HEIGHT;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], UVC_WIDTH, UVC_HEIGHT);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, UVC_HEIGHT, x1_boundary, x2_boundary, x3_boundary, NULL, NULL,
                                              NULL);

#elif (2 == INCLUDE_BOUNDARY_TYPE)
    for (i = 0; i < UVC_WIDTH; i++)
    {
        y1_boundary[i] = 50 - (50 - 20) * i / UVC_HEIGHT;
        y2_boundary[i] = UVC_WIDTH / 2;
        y3_boundary[i] = 78 - (78 - 58) * i / UVC_HEIGHT;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], UVC_WIDTH, UVC_HEIGHT);
    seekfree_assistant_camera_boundary_config(Y_BOUNDARY, UVC_WIDTH, NULL, NULL, NULL, y1_boundary, y2_boundary,
                                              y3_boundary);

#elif (3 == INCLUDE_BOUNDARY_TYPE)
    j = 0;
    for (i = UVC_HEIGHT - 1; i >= UVC_HEIGHT / 2; i--)
    {
        xy_x1_boundary[j] = 34;
        xy_y1_boundary[j] = i;
        xy_x2_boundary[j] = 47;
        xy_y2_boundary[j] = i;
        xy_x3_boundary[j] = 60;
        xy_y3_boundary[j] = i;
        j++;
    }

    for (i = UVC_HEIGHT / 2 - 1; i >= 0; i--)
    {
        xy_x1_boundary[j] = 34 + (UVC_HEIGHT / 2 - i) * (UVC_WIDTH / 2 - 34) / (UVC_HEIGHT / 2);
        xy_y1_boundary[j] = i;
        xy_x2_boundary[j] = 47 + (UVC_HEIGHT / 2 - i) * (UVC_WIDTH / 2 - 47) / (UVC_HEIGHT / 2);
        xy_y2_boundary[j] = 15 + i * 3 / 4;
        xy_x3_boundary[j] = 60 + (UVC_HEIGHT / 2 - i) * (UVC_WIDTH / 2 - 60) / (UVC_HEIGHT / 2);
        xy_y3_boundary[j] = 30 + i / 2;
        j++;
    }

    for (i = 0; i < UVC_HEIGHT / 2; i++)
    {
        xy_x1_boundary[j] = UVC_WIDTH / 2 + i * (138 - UVC_WIDTH / 2) / (UVC_HEIGHT / 2);
        xy_y1_boundary[j] = i;
        xy_x2_boundary[j] = UVC_WIDTH / 2 + i * (133 - UVC_WIDTH / 2) / (UVC_HEIGHT / 2);
        xy_y2_boundary[j] = 15 + i * 3 / 4;
        xy_x3_boundary[j] = UVC_WIDTH / 2 + i * (128 - UVC_WIDTH / 2) / (UVC_HEIGHT / 2);
        xy_y3_boundary[j] = 30 + i / 2;
        j++;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], UVC_WIDTH, UVC_HEIGHT);
    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, xy_x1_boundary, xy_x2_boundary, xy_x3_boundary,
                                              xy_y1_boundary, xy_y2, xy_y3_boundary);

#elif (4 == INCLUDE_BOUNDARY_TYPE)
    for (i = 0; i < UVC_HEIGHT; i++)
    {
        x1_boundary[i] = 70 - (70 - 20) * i / UVC_HEIGHT;
        x2_boundary[i] = UVC_WIDTH / 2;
        x3_boundary[i] = 80 + (159 - 80) * i / UVC_HEIGHT;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, NULL, UVC_WIDTH, UVC_HEIGHT);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, UVC_HEIGHT, x1_boundary, x2_boundary, x3_boundary, NULL, NULL,
                                              NULL);
#endif

    // ====================== 4. 注册清理函数 + 摄像头初始化 ======================
    atexit(cleanup);
    signal(SIGINT, sigint_handler);

    if (uvc_cam.init(UVC_PATH) < 0)
    {
        printf("Camera init failed!\r\n");
        return -1;
    }

    bool vision_ready = vision_init();
    if (!vision_ready)
    {
        printf("[VISION] init failed, vision disabled.\r\n");
    }

    printf("System init complete! Running...\r\n");

    // ====================== 5. 主循环 ======================
    static int first_frame = 1; // 第一帧标记

    while (true)
    {
        if (uvc_cam.wait_image_refresh() < 0)
        {
#ifdef DEBUG_PRINT
            printf("wait_image_refresh failed!!!\n");
#endif
            continue;
        }

        ImageProcess();
        Menu_Process();

        if (first_frame)
        {
            first_frame = 0;
            start_motor_timer(); // 允许定时器中断执行motor_control()
            printf("[SYSTEM] First frame OK, motor control ENABLED!\r\n");
        }

#ifdef DEBUG_PRINT
        printf("Det_True:%d Left:%d Right:%d WhiteLine:%d Rings:%d RoadType:%d\n", ImageStatus.Det_True,
               ImageStatus.Left_Line, ImageStatus.Right_Line, ImageStatus.WhiteLine, ImageFlag.image_element_rings_flag,
               ImageStatus.Road_type);
#endif
        uint16_t *rgb_image = uvc_cam.get_rgb_image_ptr();
        if (rgb_image != nullptr)
        {
            static int vision_frame_counter = 0;
            if (vision_ready && ++vision_frame_counter >= 10)
            {
                vision_frame_counter = 0;
                int vision_result = vision_get_from_rgb565(rgb_image, UVC_WIDTH, UVC_HEIGHT);
                static bool vision_beep_latched = false;
                if (vision_result >= 0 && vision_result <= 2)
                {
                    avoid_set_vision_result(vision_result);
                    printf("vision_get() result: %d\n", vision_result);
                    if (!vision_beep_latched)
                    {
                        beep_times(vision_result + 1);
                        vision_beep_latched = true;
                    }
                }
                else
                {
                    vision_beep_latched = false;
                }
            }
        }

        // ========== IPS200屏幕显示 ==========
        static const int SCREEN_W = 160;
        static const int SCREEN_H = 120;
        static uint16 screen_buf[SCREEN_H][SCREEN_W]; // IPS200 display buffer
        memset(screen_buf, 0, sizeof(screen_buf));

        if (rgb_image != nullptr)
        {
            // Map UVC frame into fixed 160x120 buffer; safe for 160x120 or 320x240 camera input.
            for (int y = 0; y < SCREEN_H; y++)
            {
                int src_y = y * UVC_HEIGHT / SCREEN_H;
                for (int x = 0; x < SCREEN_W; x++)
                {
                    int src_x = x * UVC_WIDTH / SCREEN_W;
                    screen_buf[y][x] = rgb_image[src_y * UVC_WIDTH + src_x];
                }
            }

            // 步骤2: 叠加边界线和轨迹线到screen_buf
            draw_boundary_on_screen(screen_buf);   // 左右红色边界
            draw_trajectory_on_screen(screen_buf); // 中线蓝色轨迹

            // 步骤2.5: 叠加横线标记到screen_buf
            draw_offline_line_on_screen(screen_buf); // 🔴丢线位置红色横线
            draw_towpoint_lines_on_screen(screen_buf); // 🟢前瞻点范围青色横线

            // 步骤3: 刷新整个IPS200屏幕（居中显示160×120）
            ips200.show_rgb565_image(80, 60, screen_buf[0], SCREEN_W, SCREEN_H, SCREEN_W, SCREEN_H, 0);
            Menu_Draw();
        }

        // ========== 第二部分：逐飞助手图传数据准备（每帧刷新一次）==========
        static int frame_counter = 0;
        if (image_transfer_state == IMAGE_TRANSFER_RUNNING && ++frame_counter >= IMAGE_TRANSFER_INTERVAL)
        {
            frame_counter = 0;
            // 准备灰度二值化图像到image_copy数组（80×60低带宽图传）
            for (int y = 0; y < LCDH; y++)
            {
                for (int x = 0; x < LCDW; x++)
                {
                    image_copy[y][x] = (Pixle[y][x] == 0) ? 0 : 255;
                }
            }

            // 叠加标注线到图传数据（用于电脑端分析）
            draw_annotation_on_imagecopy(); // 边界线(浅灰230) + 轨迹线(中灰150)

            // ========== 第三部分：发送图传数据到逐飞助手 ==========
            seekfree_assistant_camera_send();
        }
    }

    return 0;
}


// **************************** 清理函数 ****************************
void sigint_handler(int signum)
{
    printf("\n收到Ctrl+C信号，程序即将退出...\r\n");
    exit(0);
}

void cleanup()
{
    pit_timer.stop();
    vision_close();
    printf("程序退出，执行清理操作\r\n");
    motor1_pwm_1.set_duty(0); // ⚠️ 使用 t3 的变量名！
    motor2_pwm_2.set_duty(0); // ⚠️ 使用 t3 的变量名！
    printf("清理完成，电机已停止\r\n");
}
