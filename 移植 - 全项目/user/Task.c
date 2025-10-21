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
#include "usart.h"
uint16_t x,y,z;
int16_t AX, AY, AZ, GX, GY, GZ;
uint16_t Rc_Data[8];
uint8_t Cal=0;

// ��Ԫ�����ݴ洢
float quaternion_w, quaternion_x, quaternion_y, quaternion_z; 


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
		
        // ��ȡԭʼ����������
        GY86_GetData(&x, &y, &z, &AX, &AY, &AZ, &GX, &GY, &GZ);
        
        // ִ����Ԫ���ں��㷨
        Getdata();
        
        // ������Ԫ������
        quaternion_w = q[0];
        quaternion_x = q[1];
        quaternion_y = q[2];
        quaternion_z = q[3];
        
        // ����ͨ��֡��ʽ������Ԫ������
        uint8_t frame_buffer[15];  // ����֡������
        uint8_t frame_index = 0;
       
        // 1. ֡ͷ (HEAD) - 0xAB
        frame_buffer[frame_index++] = 0xAA;
        
        // 2. Դ��ַ (S_ADDR) - �豸ID��������Ϊ0x01
        frame_buffer[frame_index++] = 0xFF;
        
        // 3. Ŀ���ַ (D_ADDR) - �����豸ID��������Ϊ0x02
        frame_buffer[frame_index++] = 0x04;
      
        // 5. ���ݳ��� (LEN) - С����16�ֽ�
        frame_buffer[frame_index++] = 0x09;        // ���ֽ�
     
        // 6. �������� (DATA) - ��Ԫ�����ݣ�16�ֽڣ�
        memcpy(&frame_buffer[frame_index], &quaternion_w, 4);
        frame_index += 2;
        memcpy(&frame_buffer[frame_index], &quaternion_x, 4);
        frame_index += 2;
        memcpy(&frame_buffer[frame_index], &quaternion_y, 4);
        frame_index += 2;
        memcpy(&frame_buffer[frame_index], &quaternion_z, 4);
        frame_index += 2;
        frame_buffer[frame_index++] = (uint8_t)(0); 
        // 7. ��У�� (SUM CHECK) �� 8. ����У�� (ADD CHECK)
        // ����ͼƬ�е��㷨��SUM CHECK���ۼ�У�飬ADD CHECK�Ƕ�SUM CHECK�м�ֵ���ۼ�
        uint8_t sum_check, add_check;
        CalculateChecksums(frame_buffer, frame_index, &sum_check, &add_check);
        
        frame_buffer[frame_index++] = sum_check;
        frame_buffer[frame_index++] = add_check;
        
        // ��������֡
        BLE_SendArray(frame_buffer, 15);
        
        // ������ʱ���������ݷ���Ƶ��
        OSTimeDly(10); // ��ʱ10��ϵͳʱ�����ڣ�Լ100ms������ϵͳʱ��Ϊ100Hz��
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
