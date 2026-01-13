#ifndef __ADC_H
#define __ADC_H
#include "stm32f4xx.h"

// 电池电压相关配置
// 假设电池为3S锂电池，标称电压11.1V，满电12.6V
#define VOLTAGE_NOMINAL 11.1f 
#define ADC_REF_VOLTAGE 3.3f

// 电压分压比配置
// 警告：请根据实际硬件电路修改此值！
// V_bat = V_pin * RATIO
// NUCLEO-F401RE Pin: A4 (CN8 Pin 32) <-> PC1
// 默认假设用户连接了分压电路，请确保分压后电压不超过3.3V
// 常见分压电阻配置：
// 10k/1k -> 11.0
// 10k/2.2k -> 5.54
// 这里的默认值设为11.0，请用户务必确认硬件连接
#define VOLTAGE_DIVIDER_RATIO 11.0f 

void ADC_User_Init(void);
float Get_Battery_Voltage(void);
float Get_Voltage_Scale(void);

#endif
