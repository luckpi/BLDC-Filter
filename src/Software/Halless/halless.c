#include "halless.h"
#include "common.h"
#include "pwm.h"
#include "debug.h"
#include "uart.h"
#include "adc.h"
#include "control.h"
/*AND & OR operators for masking the active BEMF signal*/
const u8 xdata ADC_MASK[16] = {0x00, 0x04, 0x02, 0x01, 0x04, 0x02, 0x01, 0x00,  //正转
                               0x00, 0x04, 0x01, 0x02, 0x04, 0x01, 0x02, 0x00}; //反转
const u8 xdata ADC_XOR[16] = {0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00,   //正转
                              0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x00};  //反转
/*BEMF Majority Function Filter values*/
const u8 xdata ADC_BEMF_FILTER[64] =
    // 0    1    2    3      4     5     6     7     8     9     10    11    12    13    14    15
    {0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12, 0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E,
     0x20, 0x22, 0x24, 0x26, 0x28, 0x2A, 0x2C, 0x2E, 0x01, 0x01, 0x01, 0x36, 0x01, 0x3A, 0x3C, 0x3E,
     0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x01, 0x01, 0x01, 0x16, 0x01, 0x1A, 0x1C, 0x1E,
     0x01, 0x01, 0x01, 0x26, 0x01, 0x2A, 0x2C, 0x2E, 0x01, 0x01, 0x01, 0x36, 0x01, 0x3A, 0x3C, 0x3E};
/*****************************************************************************
 函 数 名  : CalcAverageSpeedTime
 功能描述  : 对转速求平均
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void CalcAvgSpeedTime()
{
    HoldParm.SpeedTime_Sum += HoldParm.SpeedTimeTemp;
    if (++HoldParm.SpeedTime_Cnt >= 8)
    {
        HoldParm.SpeedTime_Cnt = 0;
        HoldParm.SpeedTime = HoldParm.SpeedTime_Sum >> 3;
        HoldParm.SpeedTime_Sum = 0;
    }
    if (HoldParm.SpeedTime)
    {
        HoldParm.RPM = SPEEDFACTOR / HoldParm.SpeedTime;
    }
    else
    {
        HoldParm.RPM = SPEEDFACTOR / HoldParm.SpeedTimeTemp;
    }
}

static void CalcSpeedTime()
{
    SFRPAGE = 0x00;
    HoldParm.SpeedTimeTemp = (TH0 << 8) + TL0;
    TIMER0_RESET;
    if (Halless.Filter_Count <= 6 && mcState == mcRun) //堵转检测
    {
        if (++HoldParm.MainDetectCnt >= 50)
        {
            PWMPortShut();
        }
    }
    else
    {
        HoldParm.MainDetectCnt = 0;
    }
    Halless.delay_time = 65535 - (HoldParm.SpeedTimeTemp >> 1 - 100);
    TH2 = Halless.delay_time >> 8;
    TL2 = Halless.delay_time & 0xff;
    Halless.Filter_Count = 0;
    ADCSample.BackEMFFilter = 0;
    Halless.zero_flag = 1;
    CalcAvgSpeedTime();
}
void CheckZeroCrossing()
{
    u8 Num = 0;
    if (++Halless.Filter_Count >= Halless.Filter_Cnt)
    {
        ADCSample.ComparatorOutputs = 0;
        ADCSample.NeutralPoint = (ADCSample.UBemf + ADCSample.VBemf + ADCSample.WBemf) / 3;
        if (ADCSample.UBemf > ADCSample.NeutralPoint)
            ADCSample.ComparatorOutputs |= 0x01;
        if (ADCSample.VBemf > ADCSample.NeutralPoint)
            ADCSample.ComparatorOutputs |= 0x02;
        if (ADCSample.WBemf > ADCSample.NeutralPoint)
            ADCSample.ComparatorOutputs |= 0x04;
        if (HoldParm.RotorDirection == CW)
            Num = Halless.Phase;
        else
            Num = Halless.Phase + 8;
        if ((ADCSample.ComparatorOutputs ^ ADC_XOR[Num]) & ADC_MASK[Num])
        {
            ADCSample.BackEMFFilter |= 0x01;
        }
        ADCSample.BackEMFFilter = ADC_BEMF_FILTER[ADCSample.BackEMFFilter];
        if (ADCSample.BackEMFFilter & 0x01)
        {
            if (++Halless.Phase > 6)
            {
                Halless.Phase = 1;
            }
            CalcSpeedTime();
        }
    }
}
/*****************************************************************************
 函 数 名  : ThreeBemfSample
 功能描述  : 三相BEMF采样
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void ThreeBemfSample()
{
    if (ADCSample.ChlState == CHL_UBemf)
    {
        ADCSample.UBemf = Get_CHL_Value();
        ADCSample.ChlState = CHL_VBemf;
    }
    else if (ADCSample.ChlState == CHL_VBemf)
    {
        ADCSample.VBemf = Get_CHL_Value();
        ADCSample.ChlState = CHL_WBemf;
    }
    else if (ADCSample.ChlState == CHL_WBemf)
    {
        ADCSample.WBemf = Get_CHL_Value();
        ADCSample.ChlState = CHL_UBemf;
    }
}
/*****************************************************************************
 函 数 名  : ADCAnalogSample
 功能描述  : 采样，电流、电压
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void ADCAnalogSample()
{
    if (ADCSample.ChlState == CHL_UBemf)
    {
        Start_ADCSample(CHL_VBUS);
        ADCSample.Voltage = Get_CHL_Value();
    }
    else
    {
        Start_ADCSample(CHL_IBUS);
        ADCSample.Current = Get_CHL_Value();
        ADCSample.Sum += ADCSample.Current;
        if (++ADCSample.Num == 8)
        {
            ADCSample.Average = (ADCSample.Sum >> 3);
            ADCSample.Sum = 0;
            ADCSample.Num = 0;
        }
    }
    Switch_ADC_CHL(ADCSample.ChlState);
}
/*****************************************************************************
 函 数 名  : ADC_ISR
 功能描述  : ADC中断
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void ADC_ISR() ADC_IR_Num
{
    volatile u16 u16Temp = 0;
    ADC_IF_CLEAR;
    switch (mcState)
    {
    case mcDrag:
        ThreeBemfSample();
        ADCAnalogSample();
        StartupDrag();
        break;
    case mcRun:
        HoldParm.SpeedLoopCnt++;
        ThreeBemfSample();
        ADCAnalogSample();
        CheckZeroCrossing();
        break;
    default:
        break;
    }
#if (EN_UART_TX_ADC)
    //u16Temp = mcState;
    //u16Temp = HoldParm.PWMDutyCycle;
    //u16Temp = HoldParm.RequireSpeed;
    //u16Temp = ADCSample.Current;
    u16Temp = ADCSample.Voltage;
    //u16Temp = ADCSample.CurrentOffset;
    // u16Temp = ADCSample.Average;
    //u16Temp = ADCSample.Temperature;

    //u16Temp = HoldParm.RequireSpeed;

    //u16Temp = HoldParm.RotorDirection;

    //u16Temp = HallParm.HallState;
    //u16Temp = HallParm.HallUniteState;
    //u16Temp = HallParm.HallTime;
    UART_TX_EXT(u16Temp);
#endif
}