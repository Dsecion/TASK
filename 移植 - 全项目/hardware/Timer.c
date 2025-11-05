//配置TIM4中断，以供PID反馈调节使用，频率暂定400Hz
#include "stm32f4xx.h"
#include "Timer.h"
#include "ucos_ii.h"
#include "stm32f4xx_conf.h"

// 定时器相关变量
static uint32_t control_timer_count = 0;
OS_EVENT *Control_Timer_Sem;//信号量实现中断和电机控制之间的通信

/**
 * @brief  TIM4初始化 - 用于控制定时器（400Hz）
 */
void TIM4_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 使能TIM4时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    // 定时器配置：400Hz控制频率
    // 84MHz / (2100 * 100) = 400Hz
    TIM_TimeBaseStructure.TIM_Period = 100 - 1;        // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = 2100 - 1;    // 预分频器
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    // 使能TIM4更新中断
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    
    // 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 启动定时器
    TIM_Cmd(TIM4, ENABLE);
}

/**
 * @brief  重写TIM4中断服务函数 ，实现控制定时器
 */
void TIM4_IRQHandler(void)
{
    OS_CPU_SR cpu_sr = 0u;
	(void)cpu_sr;  // 显式标记为未使用
    OSIntEnter();
    
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        control_timer_count++;
        
        // 每2.5ms触发一次控制任务（400Hz）
        OSSemPost(Control_Timer_Sem);
        
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
    
    OSIntExit();
}

///**
// * @brief  获取微秒计时（用于计算dt）
// */
//uint32_t Get_Microseconds(void)
//{
//    return control_timer_count * 2500; // 400Hz，每周期2500μs
//}
