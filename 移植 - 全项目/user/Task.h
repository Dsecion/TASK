#ifndef __TASK_H_
#define __TASK_H_

#include "ucos_ii.h"

// RTOS�ź���
extern OS_EVENT *PPM_Sem;

// ң��������
extern uint16_t Rc_Data[8];

// ����������
void TASK_ShowGY86Data(void *p_arg);
void TASK_ChangeMotor(void *p_arg);

#endif /* __TASK_H_ */
