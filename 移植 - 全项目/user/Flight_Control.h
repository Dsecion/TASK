#ifndef __FLIGHT_CONTROL_H
#define __FLIGHT_CONTROL_H

#include "stm32f4xx.h"

// 外部调用接口
void Flight_Control_UpdateOuterLoop(uint16_t *rc_data);
void Flight_Control_UpdateInnerLoop(void);
void Flight_Control_ProcessSignalLoss(void); // 失控保护

#endif
