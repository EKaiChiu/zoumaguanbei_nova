#ifndef IMAGE_HPP
#define IMAGE_HPP

#include "zf_common_headfile.hpp"
#include <cstdint>

#define LCDH 60
#define LCDW 80
#define LimitL(L) (L = ((L < 1) ? 1 : L))    //限制幅度
#define LimitH(H) (H = ((H > 78) ? 78 : H))  //限制幅度
#define ImageSensorMid 49                    //图像屏幕中点
extern int ImageScanInterval;  //扫边范围    上一行的边界+-ImageScanInterval
extern int ImageScanInterval_Cross;  //十字扫边范围
extern uint8_t Image_Use[LCDH][LCDW];
extern uint8_t Pixle[LCDH][LCDW];

typedef struct {
  int point;
  uint8_t type;
} JumpPointtypedef;

typedef struct {
  float nowspeed;     // pulse表示nowspeed
  int expectspeed;    // speed表示expectspeed
  int motor_duty;     //电机占空比
  float Length;       //走过路程
  int Circle_OUT_th;
  int MinSpeed;             //最低速度
  int MaxSpeed;             //最高速度
  float expect_True_speed;  //实际期望速度
  int straight_speed;       //直道速度
} SpeedDatatypedef;

typedef struct {
  /*左右边边界标志    T为正常跳变边    W为无边   P为障碍类多跳边的内边*/
  uint8_t IsRightFind;      //右边有边标志
  uint8_t IsLeftFind;       //左边有边标志
  uint8_t isBlackFind;      //三叉边
  int Wide;               //边界宽度
  int LeftBorder;         //左边界
  int RightBorder;        //右边界
  int close_LeftBorder;   //靠边边界
  int close_RightBorder;  //靠边边界
  int opp_LeftBorder;     //反向边界
  int opp_RightBorder;    //反向边界
  int Center;             //中线
  int RightTemp;          //右边临时值
  int LeftTemp;           //左边临时值
  int CenterTemp;         //中线临时值
  int Black_Point;        //单行黑点数量

  //左右手法则扫线数据
  int LeftBoundary_First;  //左边界 保存第一次数据
  int RightBoundary_First; //右边界 保存第一次数据
  int LeftBoundary;        //左边界 保存最后一次数据
  int RightBoundary;       //右边界 保存最后一次数据

  // fork   下面的变量用于岔路口的检测
} ImageDealDatatypedef;

//元素类型
typedef enum {
  Normol,       //无任何特征
  Straight,     ////直道
  Cross,
  Ramp,         //坡道
  LeftCirque,   ////左圆环
  RightCirque,  ////右圆环
  Cross_ture,   ////十字
  Barn_in,
  Barn_out,
} RoadType_e;

typedef struct {
  /*以下关于全局图像正常参数*/

  //图像信息
  int TowPoint;             //初始比较点           给定前瞻
  int TowPointAdjust_v;     //初始比较点适应速度    没用
  int TowPoint_True;        //实际比较点           没用
  int TowPoint_Gain;        //增益系数             没用
  int TowPoint_Offset_Max;  //最大偏置系数         没用
  int TowPoint_Offset_Min;  //最小偏置系数         没用
  int Det_True;             //由GetDet()函数解算出来的平均图像偏差
  int Det_all;              //图像的近端到远端的总偏差
  float Det_all_k;          //斜率

  uint8_t Threshold;          //二值化阈值
  uint32_t Threshold_static;  //二值化静态下限
  uint8_t Threshold_detach;   //阳光算法分割阈值
  uint8_t MiddleLine;         //屏幕中心
  int Foresight;            //偏差速度因子   远端的偏差  用于控速
  uint8_t Left_Line;          //左边丢边数
  uint8_t Right_Line;         //右边丢边数
  uint8_t OFFLine;            /////图像顶边
  uint8_t WhiteLine;          ////双边丢边数
  float ExpectCur;          /////图像期望曲率    没用
  float White_Ritio;        //有效行白点比例     没用
  int Black_Pro_ALL;        //整体黑点比例%      没用

  // PID
  float Piriod_P;  //分段P   值     没用
  float MU_P;

  RoadType_e Road_type;  //元素类型

  /****圆环***/
  uint8_t IsCinqueOutIn;  //进出圆环
  uint8_t CirquePass;     //圆环中
  uint8_t CirqueOut;      //出圆环
  uint8_t CirqueOff;      //圆环结束

  //左右手法则扫线数据

  int16_t WhiteLine_L;        //左边丢线数
  int16_t WhiteLine_R;        //右边丢线数
  int16_t OFFLineBoundary;   //八领域截止行

  int Pass_Lenth;       //入环距离  用于防抖
                        /****圆环***/
  int Cirque1lenth;        //tly1
  int Cirque2lenth;        //tly2
  int Out_Lenth;     //
  int Fork_Out_Len;  //出岔口防抖
  int Dowm_lenth;    //三叉减速距离  //g5.13
  int Cross_Lenth;  // 270°转过之后一般是十字路口   作为三岔口的消抖
  int Cross_ture_lenth;
  int Sita;  //据此判断在过去一段路程中转过多少角度
  int pansancha_Lenth;

  //车库
  int Barn_Flag;   //判断库的次数
  int Barn_Lenth;  //入库停止距离
  int sanchaju;

  //保护
  int Stop_lenth;  //出界保护放误判距离
  //坡道减速
  int Ramp_lenth;

  int variance;  //直道检测阈值方差

  int straight_acc;  //直道加速标志位
  int  variance_acc;    //用于加速的阈值方差

  int ramptestlenth;//坡道检测间隔

  int rukuwait_lenth;
  int rukuwait_flag;
 // int Blue_lenth;
  int newblue_flag;

} ImageStatustypedef;

typedef struct{

  uint8_t SteerOK;      //舵机启动标志
  uint8_t CameraOK;     //摄像头启动显示标志
  uint8_t OldCameraOK;  //灰度传输标志
  uint8_t MotorOK;      //电机开关
  uint8_t Point;        //赛道实际中心
  uint8_t UpdataOK;     //数据更新
  uint8_t Stop;         //停止标志
  uint8_t GO_OK;        //冲冲冲
  int Model;          //车辆模式

  //图像参数
  int OutCicle_line;  //出环判断图像截止行  越大出环判断越严格越晚
  int L_T_R_W;  // 出环左有边右无边数量  越大出环结束越严格
  int Circleoff_offline;  //看到远处的距离判断出环结束  越小越严格
  int CircleP;            //环内P


//防误判距离
  int fork_start_lenth;  //岔路口检测起始
  int fork_off_lenth;//岔路口检测截止

  int circle_off_lenth;//出环防误判距离

  //小圆环
  int circles_pl;
  int circles_pr;
  int circles_off_lenth;//小圆环出环防误判距离

   //中圆环
  int circlem_pl;
  int circlem_pr;
  int circlem_off_lenth;//小圆环出环防误判距离

   //大圆环
  int circlel_pl;
  int circlel_pr;
  int circlel_off_lenth;//小圆环出环防误判距离
  int clrcle_priority[3];//圆环类型
  int clrcle_num;// 在第几个圆环


  int circle_kin;//入环补线半径
  float circle_kout;//出环补线斜率
  int circle_max_ang;//环内大叫限幅


  //直道
  float straight_p;//直道P
  float straight_d;//直道D
  int   straighet_towpoint;//直道前瞻


  int debug_lenth;//调试距离

  //摄像头配置
  int exp_time; //曝光时间
  int mtv_lroffset;//摄像头左右偏置
  int mtv_gain;//摄像头增益

  int ramp_lenth_start;//坡道距离
  int fork_lenth_start;//三叉距离
  int barn_lenth;//圆环距离

  int outbent_acc;//出弯加速


  int rounds; // 圈数
  int speed_per_round; // 每圈减速多少

  SpeedDatatypedef SpeedData;
} SystemDatatypdef;

typedef struct {
  int16_t image_element_rings;                  /*0 :无圆环          1 :左圆环       2 :右圆环*/
  int16_t ring_big_small;                       /*0:无                     1 :大圆环       2 :小圆环*/
  int16_t image_element_rings_flag;             /*圆环进程*/
  int16_t straight_long;                        /*长直道标志位*/
} ImageFlagtypedef;

typedef struct {
  uint16_t area;
  uint16_t left;
  uint16_t right;
  uint16_t top;
  uint16_t bottom;
  uint16_t center_x;
  uint16_t center_y;
} ROIRegionTypedef;



//int Limit(int num, int numH, int numL);
extern bool Cross_State_Print;                         // 十字路口检测打印开关
extern ImageStatustypedef ImageStatus;                //图像的全局变量
extern SystemDatatypdef SystemData;
extern ImageDealDatatypedef ImageDeal[60];                //记录单行的信息
extern ImageFlagtypedef ImageFlag;
extern ImageFlagtypedef ImageFlag;
extern ImageStatustypedef ImageData;
extern ROIRegionTypedef LargestWhiteRegion;

float my_abs(float x);
void ImageProcess(void);    ////图像处理主函数
void DrawLine();
void Element_Test(void);
void Element_Handle();
void cleanup();
void FindLargestWhiteRegion(ROIRegionTypedef* roi);

#endif
