#ifndef _PID_H
#define _PID_H
#include "common.h"
#include "typedef.h"
#define Speed_Kp 0.05f
#define Speed_Ki 0
#define Speed_Kc 0
typedef struct
{
	float InRef; // 参考
	float FB;	 // 反馈
	float Kp;
	float Ki;
	float Kc; // 与Excess相乘系数
	float Err;
	float Outmin;
	float Outmax;
	float Excess; // 抑制积分过饱和
	float Err_Sum;
	float U; // 无限输出
	float Out;
} PIDREG_T;
#define UP16LIMIT(var, max, min)               \
	{                                          \
		(var) = (var) > (max) ? (max) : (var); \
		(var) = (var) < (min) ? (min) : (var); \
	}
extern PIDREG_T xdata PI;
extern void PID_init(void);
extern void PID_CLAC(PIDREG_T *PID);
#endif
