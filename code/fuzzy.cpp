#include "fuzzy.hpp"

void Data_Settings(void) // 参数赋值
{
    // 图像参数
    // adcsum = 0;
    ImageStatus.MiddleLine = 40; // 中线（你调好的值）
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
    ImageStatus.Threshold_static = 50;  // 静态阈值（更敏感）
    ImageStatus.Threshold_detach = 150; // 阳光算法

    ImageScanInterval = 2;       // 扫边范围    上一行的边界+-ImageScanInterval
    ImageScanInterval_Cross = 2; // 十字扫线范围
    // ImageStatus.variance = 26;           //直道方差阈值
    ImageStatus.variance_acc = 25; // 直道检测
    //  SystemData.outbent_acc  =  5;
    SystemData.clrcle_num = 0;
    ImageStatus.newblue_flag = 0;
    SystemData.Stop = 1; // 启动标志位
    // BlueTooth_Flag=1;

    SteerPIDdata.LastError = 0;
    /**位置式pid参数**/
    // 重中之重： 电池电压低于12V 去充电 否则参数都是不对的
    // 方向环可以先调P 发现P已经转弯接近内切的时候，再去加D
    SteerPIDdata.P = 20; // 原来10.8 原来2.9   100hz参数      200hz参数:9.0 9.5 10.5 11.5 12.0 13.0
    //  11.0  比较适的值：10.5
    SteerPIDdata.I = 0.0;
    SteerPIDdata.D =
        80; // 原来50         9.0 10.0   目前最好9.5                               原来6.8 测试200hz参数:29.0
}