#ifndef __TIMER_H_
#define __TIMER_H_

#include "stm32f4xx.h"
#include "ucos_ii.h"

// 外部信号量声明
extern OS_EVENT *Control_Timer_Sem;

// 函数声明
void TIM4_Init(void);
//uint32_t Get_Microseconds(void);

#endif /* __TIMER_H_ */
