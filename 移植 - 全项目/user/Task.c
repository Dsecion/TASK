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
        
   //     // ִ����Ԫ���ں��㷨
    Getdata();
       
    // int16_t q_roll = (int16_t)(roll*100);
	  // int16_t q_pitch = (int16_t)(pitch*100);
	  // int16_t q_yaw = (int16_t)(yaw*100)	;
    // 
    // int8_t frame_buffer[13];  
    // uint8_t frame_index = 0;
    // 
    // // 1. ֡ͷ (HEAD) - 0xAB
    // frame_buffer[frame_index++] = 0xAA;
    // 
    // // 2. Դ��ַ (S_ADDR) - �豸ID��������Ϊ0x01
    // frame_buffer[frame_index++] = 0xFF;
    // 
    // // 3. Ŀ���ַ (D_ADDR) - �����豸ID��������Ϊ0x02
    // frame_buffer[frame_index++] = 0x03;
    // 
    // // 5. ���ݳ��� (LEN) - С����16�ֽ�
    // frame_buffer[frame_index++] = 7;        // ���ֽ�
	  // 
    // memcpy(&frame_buffer[frame_index], &q_roll, 2);
    // frame_index += 2;
    // memcpy(&frame_buffer[frame_index], &q_pitch, 2); 
    // frame_index += 2;
    // memcpy(&frame_buffer[frame_index], &q_yaw, 2);
    // frame_index += 2; 
	  // 
    // frame_buffer[frame_index++] = (uint8_t)(2); 
		// 
    // uint8_t sum_check = 0;
    // uint8_t	add_check = 0;
    // for(int i = 1; i <= 10; i++) {  // 从D_ADDR(索引1)到fusion_sta(索引12)
    //      sum_check += frame_buffer[i];
	  //     add_check += sum_check;
    // }
    // 
    // 
    //    
    // frame_buffer[frame_index++] = sum_check;
    // frame_buffer[frame_index] = add_check;
    // 
    // // ��������֡
    // BLE_SendArray(frame_buffer, 13);
	  float q_roll =roll;
		float q_pitch = pitch;
		float q_yaw = yaw;
		
		BLE_Printf("%f, %f, %f\n",q_roll,q_pitch,q_yaw);
		 
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
