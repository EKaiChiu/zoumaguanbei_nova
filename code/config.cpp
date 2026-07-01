#include "config.hpp"

void Data_Settings(void) // 参数赋值
{
    // 图像参数
    ImageStatus.MiddleLine = 40; // 中线
    ImageStatus.TowPoint_Gain = 0.2;
    ImageStatus.TowPoint_Offset_Max = 5;
    ImageStatus.TowPoint_Offset_Min = -2;
    ImageStatus.TowPointAdjust_v = 160;
    ImageStatus.Det_all_k = 0.7; // 待定自动补线斜率
    ImageStatus.CirquePass = 'F';
    ImageStatus.IsCinqueOutIn = 'F';
    ImageStatus.CirqueOut = 'F';
    ImageStatus.CirqueOff = 'F';
    ImageStatus.Barn_Flag = 0;
    ImageStatus.straight_acc = 0;

    ImageStatus.TowPoint = 20;          // 前瞻（提前看远，早减速）
    ImageStatus.OFFLine = 3;            // 起始行（更早开始检测）
    ImageStatus.Threshold_static = 50;  // 静态阈值
    ImageStatus.Threshold_detach = 150; // 阳光算法

    ImageScanInterval = 2;       // 扫边范围
    ImageScanInterval_Cross = 2; // 十字扫线范围
    ImageStatus.variance_acc = 25; // 直道检测
    SystemData.clrcle_num = 0;
    ImageStatus.newblue_flag = 0;
    SystemData.Stop = 1; // 启动标志位

    SteerPIDdata.LastError = 0;
    // 方向环 PID 参数
    // 电池电压低于12V 去充电，否则参数不准
    SteerPIDdata.P = 20;
    SteerPIDdata.I = 0.0;
    SteerPIDdata.D = 80;
}
