#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"

extern TIM_HandleTypeDef TIM1_Handler;      //¶¨Ê±Æ÷¾ä±ú 

void TIM1_Init(u16 arr,u16 psc);
void TIM1_PWM_Init(u16 arr,u16 psc);
void TIM_SetTIM1Compare1(u32 compare);
#endif

