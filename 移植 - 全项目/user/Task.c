/******************************************************************************
 * ????Task.c
 * ???RTOS????
 ******************************************************************************/

#include "Task.h"
#include <ucos_ii.h>
#include <os.h>
#include "stm32f4xx.h"
#include "PID.h"
#include "PWM.h"
#include "getdata.h"
#include "Flight_Control.h"
#include "ATKBLE01.h"

// ????????
uint16_t Rc_Data[8];
static uint8_t Cal = 0;

/**
 * @brief  TIM2?????? - PPM????
 * @note   ??8??PPM?????????????
 */
void TIM2_IRQHandler(void)
{
    OS_CPU_SR cpu_sr = 0u;
    OSIntEnter();

    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
    {
        uint32_t Rc_Val = TIM_GetCapture1(TIM2);
        
        OS_ENTER_CRITICAL();
        
        // ?????????2050us????????
        if (Rc_Val > 2050) {
            Cal = 0;
        }
        else if (Cal < 8) {
            Rc_Data[Cal] = Rc_Val;
            Cal++;
            
            // 8????????????????
            if (Cal == 8) {
                OSSemPost(PPM_Sem);
                Cal = 0;
            }
        }
        
        OS_EXIT_CRITICAL();
        
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
    }
    
    OSIntExit();
}

/**
 * @brief  ???????????
 * @param  p_arg: ?????????
 * @note   ??????????Madgwick????????????
 */
void TASK_ShowGY86Data(void *p_arg)
{
    uint8_t tx_buffer[15];
    int16_t v0, v1, v2, v3;
    uint8_t sum_check, add_check;
    
    (void)p_arg;
    
    while (1)
    {
        // ??????????Madgwick????
        Getdata();
        
        // ?????????
        tx_buffer[0] = 0xAA;  // ??
        tx_buffer[1] = 0xFF;  // ????????
        tx_buffer[2] = 0x04;  // ????????
        tx_buffer[3] = 9;     // ????
        
        // ???????int16_t???10000??
        v0 = (int16_t)(q[0] * 10000.0f);
        v1 = (int16_t)(q[1] * 10000.0f);
        v2 = (int16_t)(q[2] * 10000.0f);
        v3 = (int16_t)(q[3] * 10000.0f);
        
        // ?????????
        tx_buffer[4] = (uint8_t)(v0 & 0xFF);
        tx_buffer[5] = (uint8_t)(v0 >> 8);
        tx_buffer[6] = (uint8_t)(v1 & 0xFF);
        tx_buffer[7] = (uint8_t)(v1 >> 8);
        tx_buffer[8] = (uint8_t)(v2 & 0xFF);
        tx_buffer[9] = (uint8_t)(v2 >> 8);
        tx_buffer[10] = (uint8_t)(v3 & 0xFF);
        tx_buffer[11] = (uint8_t)(v3 >> 8);
        
        // ????
        tx_buffer[12] = 1;
        
        // ?????
        sum_check = 0;
        add_check = 0;
        for (int i = 0; i < 13; i++) {
            sum_check += tx_buffer[i];
            add_check += sum_check;
        }
        
        tx_buffer[13] = sum_check;
        tx_buffer[14] = add_check;
        
        // ????
        BLE_SendArray(tx_buffer, 15);
    }
}

/**
 * @brief  ??????
 * @param  p_arg: ?????????
 * @note   ??PPM???????????
 */
void TASK_ChangeMotor(void *p_arg)
{
    INT8U err;
    
    (void)p_arg;
    
    // ?????
    PPM_Sem = OSSemCreate(0);
    
    // ???PID???
    PID_Controllers_Init();
    
    while (1)
    {
        // ??PPM???8???????
        OSSemPend(PPM_Sem, 0, &err);
        
        if (err == OS_ERR_NONE) {
            // ??????
            Flight_Control(Rc_Data);
        }
    }
}