#ifndef __capture__
#define __capture__
#include "main.h"

void TIM2_Cap_Init(u32 arr,u16 psc);
void GetCaptureVal(long long *CH1, long long  *CH2, long long  *CH3, long long  *CH4);
#endif

