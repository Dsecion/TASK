#ifndef __PWM_H_
#define __PWM_H_
#include "stm32f4xx.h"                  // Device header

void PWM_Init(void);

void PWM_SetCompareAll(uint16_t Compare);

// 独立设置四个电机的PWM值
void PWM_SetMotor1(uint16_t Compare);
void PWM_SetMotor2(uint16_t Compare);
void PWM_SetMotor3(uint16_t Compare);
void PWM_SetMotor4(uint16_t Compare);

// 电机混控函数
void PWM_Motor_Mixing(float throttle, float roll_output, float pitch_output, float yaw_output);

#endif
