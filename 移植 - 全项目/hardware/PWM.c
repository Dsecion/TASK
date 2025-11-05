#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "PID_Config.h"


void PWM_Init(void)
	{

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 84 -1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 20000 -1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);

	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	TIM_OC3Init(TIM3,&TIM_OCInitStructure);
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE); 
	TIM_Cmd(TIM3,ENABLE);

}


void PWM_SetCompareAll(uint16_t Compare)
{
	TIM_SetCompare1(TIM3, Compare);
	TIM_SetCompare2(TIM3, Compare);
	TIM_SetCompare3(TIM3, Compare);
	TIM_SetCompare4(TIM3, Compare);
}

// 电机1 - TIM3_CH1 (PA6) - 前右
void PWM_SetMotor1(uint16_t Compare)
{
	if(Compare > MOTOR_MAX) Compare = MOTOR_MAX;
	if(Compare < MOTOR_MIN) Compare = MOTOR_MIN;
	TIM_SetCompare1(TIM3, Compare);
}

// 电机2 - TIM3_CH2 (PA7) - 后右
void PWM_SetMotor2(uint16_t Compare)
{
	if(Compare > MOTOR_MAX) Compare = MOTOR_MAX;
	if(Compare < MOTOR_MIN) Compare = MOTOR_MIN;
	TIM_SetCompare2(TIM3, Compare);
}

// 电机3 - TIM3_CH3 (PB0) - 后左
void PWM_SetMotor3(uint16_t Compare)
{
	if(Compare > MOTOR_MAX) Compare = MOTOR_MAX;
	if(Compare < MOTOR_MIN) Compare = MOTOR_MIN;
	TIM_SetCompare3(TIM3, Compare);
}

// 电机4 - TIM3_CH4 (PB1) - 前左
void PWM_SetMotor4(uint16_t Compare)
{
	if(Compare > MOTOR_MAX) Compare = MOTOR_MAX;
	if(Compare < MOTOR_MIN) Compare = MOTOR_MIN;
	TIM_SetCompare4(TIM3, Compare);
}

/**
 * @brief  X型四旋翼电机混控
 * @param  throttle: 基础油门值
 * @param  roll_output: Roll轴PID输出
 * @param  pitch_output: Pitch轴PID输出
 * @param  yaw_output: Yaw轴PID输出
 * @note   电机布局:
 *         M4   M1
 *           ╲ ╱ 
 *            X   
 *           ╱ ╲  
 *         M3   M2
 */
void PWM_Motor_Mixing(float throttle, float roll_output, float pitch_output, float yaw_output)
{
    int16_t motor1, motor2, motor3, motor4;
    
    // X型四旋翼混控算法
    // M1(前右): Roll-, Pitch+, Yaw- (顺时针)
    // M2(后右): Roll-, Pitch-, Yaw+ (逆时针)
    // M3(后左): Roll+, Pitch-, Yaw- (顺时针)
    // M4(前左): Roll+, Pitch+, Yaw+ (逆时针)
    
    motor1 = (int16_t)(throttle - roll_output + pitch_output - yaw_output);
    motor2 = (int16_t)(throttle - roll_output - pitch_output + yaw_output);
    motor3 = (int16_t)(throttle + roll_output - pitch_output - yaw_output);
    motor4 = (int16_t)(throttle + roll_output + pitch_output + yaw_output);
    
    // 输出到电机
    PWM_SetMotor1((uint16_t)motor1);
    PWM_SetMotor2((uint16_t)motor2);
    PWM_SetMotor3((uint16_t)motor3);
    PWM_SetMotor4((uint16_t)motor4);
}
