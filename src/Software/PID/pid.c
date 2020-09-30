#include "pid.h"
#include "pwm.h"
#include "common.h"
PIDREG_T xdata PI;
void PID_init(void)
{
    PI.InRef = HoldParm.Set_RPM;
    PI.FB = 0;
    PI.Kp = Speed_Kp;
    PI.Ki = Speed_Ki;
    PI.Kc = Speed_Kc; // 与Excess相乘系数
    PI.Err = 0;
    PI.Outmax = 5;
    PI.Outmin = -PI.Outmax;
    PI.Excess = 0; // 抑制积分过饱和
    PI.Err_Sum = 0;
    PI.U = 0; // 无限输出
    PI.Out = 0;
}
void PID_CLAC(PIDREG_T *PI)
{
    PI->Err = PI->InRef - PI->FB;
    PI->U = PI->Err_Sum + PI->Kp * PI->Err;

    if (PI->U > PI->Outmax)
        PI->Out = PI->Outmax;
    else if (PI->U < PI->Outmin)
        PI->Out = PI->Outmin;
    else
        PI->Out = PI->U;

    PI->Excess = PI->U - PI->Out;
    PI->Err_Sum += (PI->Ki * PI->Err) - (PI->Kc * PI->Excess);
}