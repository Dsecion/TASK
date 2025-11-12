/******************************************************************************
 * 文件名：Flight_Control.h  
 * 功能：飞行控制函数声明
 ******************************************************************************/

#ifndef __FLIGHT_CONTROL_H_
#define __FLIGHT_CONTROL_H_

#include "stm32f4xx.h"

// 外环姿态更新（遥控器输入较低频率）
void Flight_Control_UpdateOuterLoop(uint16_t *rc_data);

// 内环角速度控制（传感器更新高频率）
void Flight_Control_UpdateInnerLoop(void);

#endif /* __FLIGHT_CONTROL_H_ */
