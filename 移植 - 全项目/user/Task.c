//Task.c�ļ������÷����������ȼ����
#include "Task.h"
#include <ucos_ii.h>
#include <os.h>
#include "stm32f4xx.h"                  // Device header
#include "GY86.h"
#include "OLED.h"
#include "PWM.h"
#include "getdata.h"
#include "ATKBLE01.h"
#include <string.h>
#include "Delay.h"

uint16_t Rc_Data[8];
uint8_t Cal=0;



// ��Ԫ�����ݴ洢
int16_t quaternion_w, quaternion_x, quaternion_y, quaternion_z; 


void TIM2_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr = 0u;
    OSIntEnter();  // �����ж�

    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
    {
        uint32_t Rc_Val = TIM_GetCapture1(TIM2);
        
        //������Ҫ������������
        OS_ENTER_CRITICAL();
        if(Rc_Val > 2050){
            Cal = 0;
        }
        else if(Cal < 8){
            Rc_Data[Cal] = Rc_Val;
            Cal++;
            
            // ��8ͨ����������ʱ�����ź���
            if(Cal == 8){
                OSSemPost(PPM_Sem);  // ֪ͨ����
				Cal=0;//���ü�����
            }
        }
        OS_EXIT_CRITICAL();
        
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
    }
    
    OSIntExit();  // �˳��ж�
}




void TASK_ShowGY86Data(void *p_arg){
    while(1){
        Getdata();
       
        int16_t q_roll = (int16_t)(roll*100.0f);
        int16_t q_pitch = (int16_t)(pitch*100.0f);
        int16_t q_yaw = (int16_t)(yaw*100.0f);
       
        int8_t frame_buffer[13];  // 改为uint8_t
        uint8_t sum_check = 0, add_check = 0;
        uint8_t data_len = 7;
     
        // 修正帧结构顺序
        frame_buffer[0] = 0xAA;     // 帧头
        frame_buffer[1] = 0xFF;     // 目标地址 (广播)
        frame_buffer[2] = 0x03;     // 数据ID
        frame_buffer[3] = data_len; // 数据长度
     
        // 数据部分保持不变
       frame_buffer[4] = q_roll & 0xff;     // 帧头
        frame_buffer[5] = (q_roll>>8)&0xff;     // 目标地址 (广播)
        frame_buffer[6] = q_pitch & 0xff;     // 数据ID
        frame_buffer[7] = (q_pitch>>8)&0xff;
		frame_buffer[8] = q_yaw& 0xff;     // 数据ID
        frame_buffer[9] = (q_yaw>>8)&0xff;
		 frame_buffer[10] = 2;
		// FUSION_STA改为2
     
        // 修正校验和计算
        for(int i = 0; i < 4 + data_len; i++) { // i=0到10
            sum_check += frame_buffer[i];
            add_check += sum_check;
        }
        frame_buffer[11] = sum_check;
        frame_buffer[12] = add_check;
       
        BLE_SendArray(frame_buffer, 13);
		Delay_ms(10);
    }
}

void TASK_ChangeMotor(void *p_arg){
	
    PPM_Sem = OSSemCreate(0);
    while(1){
		INT8U err;
        // �ȴ��ź���
        OSSemPend(PPM_Sem, 0, &err); 
        // ��ȡ���ݷ���Ȩ
        uint16_t throttle = Rc_Data[1];  //ͨ��1��������
        // ���õ��ת��
		if(err==OS_ERR_NONE){
			
			PWM_SetCompareAll(throttle);
		}
    }
	
}
