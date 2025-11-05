#ifndef __TASK_H_
#define __TASK_H_

#include "ucos_ii.h"
#include "stdint.h"

// RTOS信号量
extern OS_EVENT *PPM_Sem;

// 遥控器数据
extern uint16_t Rc_Data[8];

// 任务函数声明
void TASK_ShowGY86Data(void *p_arg);
void TASK_ChangeMotor(void *p_arg);

#endif /* __TASK_H_ */
