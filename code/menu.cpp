#include "menu.hpp"
#include "Key.hpp"
#include "config.hpp"



#include "zf_common_headfile.hpp"

/*
    按键说明：

    调车模式：
        KEY1：上一项 / 编辑时减小
        KEY2：下一项 / 编辑时增大
        KEY3：进入或退出编辑
        KEY4：发车 / 停车

    比赛模式：
        KEY4：发车 / 停车
        KEY1 / KEY2 / KEY3 不处理

    图像测试模式：
        只处理图像，电机不动
*/

typedef enum
{
    MENU_RUN_SPEED = 0,
    MENU_AIM_Y,

    MENU_SPEED_KP,
    MENU_SPEED_KI,
    MENU_SPEED_KD,

    MENU_TRACK_KP,
    MENU_TRACK_KI,
    MENU_TRACK_KD,

    MENU_DIFF_GAIN,
    MENU_DIFF_LIMIT,

    MENU_ITEM_COUNT
} MenuItem;

static int menu_enabled = 1;
static int menu_editing = 0;
static int menu_index = MENU_RUN_SPEED;
static int car_started = 0;

/*
函数名称：static int limit_int_local(int value, int min_value, int max_value)
功能说明：限制整数范围
参数说明：
    value：输入值
    min_value：最小值
    max_value：最大值
函数返回：限幅后的值
修改时间：2026年5月29日
备注：
example：  limit_int_local(value, 0, 800);
 */
static int limit_int_local(int value, int min_value, int max_value)
{
    if(value < min_value)
    {
        value = min_value;
    }

    if(value > max_value)
    {
        value = max_value;
    }

    return value;
}

/*
函数名称：static double limit_double_local(double value, double min_value, double max_value)
功能说明：限制浮点数范围
参数说明：
    value：输入值
    min_value：最小值
    max_value：最大值
函数返回：限幅后的值
修改时间：2026年5月29日
备注：
example：  limit_double_local(value, 0.0, 10.0);
 */
static double limit_double_local(double value, double min_value, double max_value)
{
    if(value < min_value)
    {
        value = min_value;
    }

    if(value > max_value)
    {
        value = max_value;
    }

    return value;
}

/*
函数名称：static int double_to_int_100(double value)
功能说明：浮点数放大100倍显示
参数说明：
    value：输入浮点数
函数返回：放大后的整数
修改时间：2026年5月29日
备注：
example：  double_to_int_100(car_config.diff_gain);
 */
static int double_to_int_100(double value)
{
    if(value >= 0)
    {
        return (int)(value * 100.0 + 0.5);
    }

    return (int)(value * 100.0 - 0.5);
}

/*
函数名称：static int double_to_int_1000(double value)
功能说明：浮点数放大1000倍显示
参数说明：
    value：输入浮点数
函数返回：放大后的整数
修改时间：2026年5月29日
备注：
example：  double_to_int_1000(car_config.speed_ki);
 */
static int double_to_int_1000(double value)
{
    if(value >= 0)
    {
        return (int)(value * 1000.0 + 0.5);
    }

    return (int)(value * 1000.0 - 0.5);
}

/*
函数名称：static void menu_limit_config(void)
功能说明：限制菜单参数，避免调到异常值
参数说明：无
函数返回：无
修改时间：2026年5月29日
备注：
example：  menu_limit_config();
 */
static void menu_limit_config(void)
{
    car_config.run_speed = limit_int_local(car_config.run_speed, 0, 800);
    car_config.aim_y = limit_int_local(car_config.aim_y, 20, 115);

    car_config.speed_kp = limit_double_local(car_config.speed_kp, 0.0, 100.0);
    car_config.speed_ki = limit_double_local(car_config.speed_ki, 0.0, 5.0);
    car_config.speed_kd = limit_double_local(car_config.speed_kd, 0.0, 100.0);

    car_config.track_kp = limit_double_local(car_config.track_kp, 0.0, 20.0);
    car_config.track_ki = limit_double_local(car_config.track_ki, 0.0, 5.0);
    car_config.track_kd = limit_double_local(car_config.track_kd, 0.0, 20.0);

    car_config.diff_gain = limit_double_local(car_config.diff_gain, 0.1, 20.0);
    car_config.diff_speed_lim = limit_double_local(car_config.diff_speed_lim, 50.0, 1000.0);
}

/*
函数名称：static void menu_apply_speed_pid(void)
功能说明：应用速度PID参数
参数说明：无
函数返回：无
修改时间：2026年5月29日
备注：
    修改速度PID后需要重新写入 RSPID 和 LSPID。
example：  menu_apply_speed_pid();
 */
static void menu_apply_speed_pid(void)
{
    pid_init(&RSPID,
             car_config.speed_kp,
             car_config.speed_ki,
             car_config.speed_kd);

    pid_init(&LSPID,
             car_config.speed_kp,
             car_config.speed_ki,
             car_config.speed_kd);
}

/*
函数名称：static void menu_apply_track_pid(void)
功能说明：应用巡线PID参数
参数说明：无
函数返回：无
修改时间：2026年5月29日
备注：
example：  menu_apply_track_pid();
 */
static void menu_apply_track_pid(void)
{
    trackline_init(car_config.track_kp,
                   car_config.track_ki,
                   car_config.track_kd);
}

/*
函数名称：static double menu_get_step(void)
功能说明：获取当前参数步进
参数说明：无
函数返回：当前菜单项步进
修改时间：2026年5月29日
备注：
example：  menu_get_step();
 */
static double menu_get_step(void)
{
    if(menu_index == MENU_RUN_SPEED)
    {
        return 20.0;
    }
    else if(menu_index == MENU_AIM_Y)
    {
        return 1.0;
    }
    else if(menu_index == MENU_SPEED_KP)
    {
        return 0.5;
    }
    else if(menu_index == MENU_SPEED_KI)
    {
        return 0.001;
    }
    else if(menu_index == MENU_SPEED_KD)
    {
        return 0.05;
    }
    else if(menu_index == MENU_TRACK_KP)
    {
        return 0.1;
    }
    else if(menu_index == MENU_TRACK_KI)
    {
        return 0.001;
    }
    else if(menu_index == MENU_TRACK_KD)
    {
        return 0.05;
    }
    else if(menu_index == MENU_DIFF_GAIN)
    {
        return 0.1;
    }
    else if(menu_index == MENU_DIFF_LIMIT)
    {
        return 20.0;
    }

    return 1.0;
}

/*
函数名称：static void menu_value_change(int dir)
功能说明：修改当前选中的参数
参数说明：
    dir：方向，1增加，-1减小
函数返回：无
修改时间：2026年5月29日
备注：
example：  menu_value_change(1);
 */
static void menu_value_change(int dir)
{
    double step = menu_get_step();

    if(menu_index == MENU_RUN_SPEED)
    {
        car_config.run_speed += (int)(dir * step);
    }
    else if(menu_index == MENU_AIM_Y)
    {
        car_config.aim_y += (int)(dir * step);
    }
    else if(menu_index == MENU_SPEED_KP)
    {
        car_config.speed_kp += dir * step;
    }
    else if(menu_index == MENU_SPEED_KI)
    {
        car_config.speed_ki += dir * step;
    }
    else if(menu_index == MENU_SPEED_KD)
    {
        car_config.speed_kd += dir * step;
    }
    else if(menu_index == MENU_TRACK_KP)
    {
        car_config.track_kp += dir * step;
    }
    else if(menu_index == MENU_TRACK_KI)
    {
        car_config.track_ki += dir * step;
    }
    else if(menu_index == MENU_TRACK_KD)
    {
        car_config.track_kd += dir * step;
    }
    else if(menu_index == MENU_DIFF_GAIN)
    {
        car_config.diff_gain += dir * step;
    }
    else if(menu_index == MENU_DIFF_LIMIT)
    {
        car_config.diff_speed_lim += dir * step;
    }

    menu_limit_config();

    if(menu_index == MENU_SPEED_KP ||
       menu_index == MENU_SPEED_KI ||
       menu_index == MENU_SPEED_KD)
    {
        menu_apply_speed_pid();
    }

    if(menu_index == MENU_TRACK_KP ||
       menu_index == MENU_TRACK_KI ||
       menu_index == MENU_TRACK_KD)
    {
        menu_apply_track_pid();
    }
}

/*
函数名称：static const char *menu_get_name(int index)
功能说明：获取菜单项名称
参数说明：
    index：菜单项编号
函数返回：菜单项名称
修改时间：2026年5月29日
备注：
example：  menu_get_name(menu_index);
 */
static const char *menu_get_name(int index)
{
    if(index == MENU_RUN_SPEED)
    {
        return "RUN";
    }
    else if(index == MENU_AIM_Y)
    {
        return "AIM";
    }
    else if(index == MENU_SPEED_KP)
    {
        return "SKP";
    }
    else if(index == MENU_SPEED_KI)
    {
        return "SKI";
    }
    else if(index == MENU_SPEED_KD)
    {
        return "SKD";
    }
    else if(index == MENU_TRACK_KP)
    {
        return "TKP";
    }
    else if(index == MENU_TRACK_KI)
    {
        return "TKI";
    }
    else if(index == MENU_TRACK_KD)
    {
        return "TKD";
    }
    else if(index == MENU_DIFF_GAIN)
    {
        return "DGAIN";
    }
    else if(index == MENU_DIFF_LIMIT)
    {
        return "DLIM";
    }

    return "NONE";
}

/*
函数名称：static int menu_get_show_value(int index)
功能说明：获取菜单显示值
参数说明：
    index：菜单项编号
函数返回：用于屏幕显示的整数
修改时间：2026年5月29日
备注：
    KP、KD、DGAIN 显示为实际值 * 100
    KI 显示为实际值 * 1000
example：  menu_get_show_value(menu_index);
 */
static int menu_get_show_value(int index)
{
    if(index == MENU_RUN_SPEED)
    {
        return car_config.run_speed;
    }
    else if(index == MENU_AIM_Y)
    {
        return car_config.aim_y;
    }
    else if(index == MENU_SPEED_KP)
    {
        return double_to_int_100(car_config.speed_kp);
    }
    else if(index == MENU_SPEED_KI)
    {
        return double_to_int_1000(car_config.speed_ki);
    }
    else if(index == MENU_SPEED_KD)
    {
        return double_to_int_100(car_config.speed_kd);
    }
    else if(index == MENU_TRACK_KP)
    {
        return double_to_int_100(car_config.track_kp);
    }
    else if(index == MENU_TRACK_KI)
    {
        return double_to_int_1000(car_config.track_ki);
    }
    else if(index == MENU_TRACK_KD)
    {
        return double_to_int_100(car_config.track_kd);
    }
    else if(index == MENU_DIFF_GAIN)
    {
        return double_to_int_100(car_config.diff_gain);
    }
    else if(index == MENU_DIFF_LIMIT)
    {
        return (int)car_config.diff_speed_lim;
    }

    return 0;
}

/*
函数名称：static int menu_get_show_step(int index)
功能说明：获取步进显示值
参数说明：
    index：菜单项编号
函数返回：用于屏幕显示的步进
修改时间：2026年5月29日
备注：
example：  menu_get_show_step(menu_index);
 */
static int menu_get_show_step(int index)
{
    double step = menu_get_step();

    if(index == MENU_SPEED_KI || index == MENU_TRACK_KI)
    {
        return double_to_int_1000(step);
    }

    if(index == MENU_RUN_SPEED ||
       index == MENU_AIM_Y ||
       index == MENU_DIFF_LIMIT)
    {
        return (int)step;
    }

    return double_to_int_100(step);
}

/*
函数名称：static void menu_show_mark(int x, int y, int selected, int editing)
功能说明：显示菜单光标
参数说明：
    x：横坐标
    y：纵坐标
    selected：是否选中
    editing：是否正在编辑
函数返回：无
修改时间：2026年5月29日
备注：
    Y：选中
    R：编辑中
example：  menu_show_mark(0, y, 1, menu_editing);
 */
static void menu_show_mark(int x, int y, int selected, int editing)
{
    if(!selected)
    {
        ips200.show_string(x, y, " ");
        return;
    }

    if(editing)
    {
        ips200.show_string(x, y, "R");
    }
    else
    {
        ips200.show_string(x, y, "Y");
    }
}

/*
函数名称：void menu_init(void)
功能说明：初始化菜单
参数说明：无
函数返回：无
修改时间：2026年5月29日
备注：
example：  menu_init();
 */
void menu_init(void)
{
    menu_enabled = 1;
    menu_editing = 0;
    menu_index = MENU_RUN_SPEED;
    car_started = 0;

    menu_limit_config();
    menu_apply_speed_pid();
    menu_apply_track_pid();
}

/*
函数名称：void menu_process(void)
功能说明：处理按键和发车状态
参数说明：无
函数返回：无
修改时间：2026年5月29日
备注：
example：  menu_process();
 */
void menu_process(void)
{
    Key_Process();

    int key = Key_GetValueOnce();

    if(key == KEY_NONE)
    {
        return;
    }

    /*
        图像测试模式只看图像，不允许发车。
     */
    if(config_is_image_test_mode())
    {
        menu_car_stop();
        return;
    }

    /*
        KEY4 在比赛模式和调车模式下都是发车 / 停车。
     */
    if(key == KEY_4)
    {
        if(car_started)
        {
            menu_car_stop();
        }
        else
        {
            menu_editing = 0;
            car_started = 1;
        }

        return;
    }

    /*
        比赛模式不调参数。
     */
    if(config_is_race_mode())
    {
        return;
    }

    /*
        发车后不再调参数。
     */
    if(car_started)
    {
        return;
    }

    if(key == KEY_3)
    {
    if(menu_editing)
    {
        config_save();
    }

    menu_editing = !menu_editing;
    return;
    }

    if(menu_editing)
    {
        if(key == KEY_1)
        {
            menu_value_change(-1);
        }
        else if(key == KEY_2)
        {
            menu_value_change(1);
        }
    }
    else
    {
        if(key == KEY_1)
        {
            menu_index--;

            if(menu_index < 0)
            {
                menu_index = MENU_ITEM_COUNT - 1;
            }
        }
        else if(key == KEY_2)
        {
            menu_index++;

            if(menu_index >= MENU_ITEM_COUNT)
            {
                menu_index = 0;
            }
        }
    }
}

/*
函数名称：int menu_is_enabled(void)
功能说明：获取菜单显示状态
参数说明：无
函数返回：
    1：显示
    0：不显示
修改时间：2026年5月29日
备注：
example：  menu_is_enabled();
 */
int menu_is_enabled(void)
{
    return menu_enabled;
}

/*
函数名称：int menu_is_editing(void)
功能说明：获取菜单编辑状态
参数说明：无
函数返回：
    1：编辑中
    0：选择中
修改时间：2026年5月29日
备注：
example：  menu_is_editing();
 */
int menu_is_editing(void)
{
    return menu_editing;
}

/*
函数名称：int menu_car_is_started(void)
功能说明：获取发车状态
参数说明：无
函数返回：
    1：已发车
    0：未发车
修改时间：2026年5月29日
备注：
example：  menu_car_is_started();
 */
int menu_car_is_started(void)
{
    return car_started;
}

/*
函数名称：void menu_car_stop(void)
功能说明：停车并回到非编辑状态
参数说明：无
函数返回：无
修改时间：2026年5月29日
备注：
example：  menu_car_stop();
 */
void menu_car_stop(void)
{
    car_started = 0;
    menu_enabled = 1;
    menu_editing = 0;

    pid_speed_set_target(0, 0);

    drv8701e_pwm_1.set_duty(0);
    drv8701e_pwm_2.set_duty(0);
}

/*
函数名称：void menu_show(void)
功能说明：显示调参菜单
参数说明：无
函数返回：无
修改时间：2026年5月29日
备注：
example：  menu_show();
 */
void menu_show(void)
{
    if(!menu_enabled)
    {
        return;
    }

    ips200.show_string(0, 0, "MENU");

    ips200.show_string(50, 0, "RUN:");
    ips200.show_int(90, 0, car_started, 1);

    ips200.show_string(110, 0, "EDIT:");
    ips200.show_int(155, 0, menu_editing, 1);

    int page_start = 0;

    if(menu_index >= 6)
    {
        page_start = menu_index - 5;
    }

    for(int i = 0; i < 6; i++)
    {
        int item = page_start + i;

        if(item >= MENU_ITEM_COUNT)
        {
            break;
        }

        int y = 25 + i * 20;

        menu_show_mark(0, y, item == menu_index, menu_editing);

        ips200.show_string(15, y, menu_get_name(item));

        ips200.show_string(75, y, ":");
        ips200.show_int(85, y, menu_get_show_value(item), 5);

        ips200.show_string(135, y, "S:");
        ips200.show_int(155, y, menu_get_show_step(item), 4);
    }

    ips200.show_string(0, 150, "K1 UP/- K2 DN/+");
    ips200.show_string(0, 170, "K3 OK  K4 START");
    ips200.show_string(0, 190, "x100:KP KD DG");
    ips200.show_string(0, 210, "x1000:KI");
}

/*
函数名称：void menu_show_run_state(void)
功能说明：显示运行状态
参数说明：无
函数返回：无
修改时间：2026年5月29日
备注：
    该函数只显示状态，不参与调参。
example：  menu_show_run_state();
 */
void menu_show_run_state(void)
{

}