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
#include "PID.h"
#include "Flight_Control.h"

uint16_t Rc_Data[8];
volatile uint8_t Cal=0;


// 在文件开头声明全局变量
volatile uint32_t fault_pc = 0;
volatile uint32_t fault_lr = 0;
volatile uint32_t fault_cfsr = 0;

// ��Ԫ�����ݴ洢
int16_t quaternion_w, quaternion_x, quaternion_y, quaternion_z; 


void TIM2_IRQHandler(void)
{
	//OS_CPU_SR  cpu_sr = 0u;
    OSIntEnter();  // �����ж�

    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
    {
        uint32_t Rc_Val = TIM_GetCapture1(TIM2);
			
			       if (Rc_Val > 3000) 
        {
            // 同步脉冲：完成一帧
            if (Cal >= 8) 
            {
                // 完整接收到8个通道，发送信号量
                OSSemPost(PPM_Sem);
            }
            Cal = 0;  // 重置通道计数器
        }
        else 
        {
            // 通道脉冲
            if (Cal < 8) 
            {
                // 范围检查：正常PPM值在1000-2000之间
                if (Rc_Val >= 800 && Rc_Val <= 2200) 
                {
                    Rc_Data[Cal] = Rc_Val;
                }
                else 
                {
                    // 异常值，使用上一次的值或默认值
                    Rc_Data[Cal] = 1500;  // 中位值
                }
                Cal++;
            }
						
            // 如果Cal >= 8，忽略多余的脉冲（可能是干扰）
        }
				TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
        
       // //������Ҫ������������
       // OS_ENTER_CRITICAL();
       // if(Rc_Val > 2050){
       //     Cal = 0;
       // }
       // else if(Cal < 8){
       //     Rc_Data[Cal] = Rc_Val;
       //     Cal++;
       //     
       //     // ��8ͨ����������ʱ�����ź���
       //     if(Cal == 8){
       //         OSSemPost(PPM_Sem);  // ֪ͨ����
				//Cal=0;//���ü�����
       //     }
       // }
       // OS_EXIT_CRITICAL();
       // 
       // TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
    }
    
    OSIntExit();  // �˳��ж�
}




void TASK_ShowGY86Data(void *p_arg){
    
    while(1){
		Getdata();
    
  //uint8_t tx_buffer[15];
  //tx_buffer[0] = 0xAA;
	//
	//tx_buffer[1] = 0xFF; // Ŀ±굘ַ (0xFF Ϊ¹㲥) [cite: 90, 426]
	//tx_buffer[2] = 0x04; // ¹¦Ĝ« (·ɿؗˌ¬:˄Ԫʽ¸񊽩
	//tx_buffer[3] = 9;	 // ʽ¾ݳ¤¶Ƞ(V0,V1,V2,V3,FUSION_STA)
	//
	//// 3. ̮³䊽¾݄ڈݠ(DATAǸ)
	//
	//// a. ½« float ת»»Ϊ int16_t (°´ЭҩÀ©´󱰰00±¶)
	//int16_t v0 = (int16_t)(q[0] * 10000.0f);
	//int16_t v1 = (int16_t)(q[1] * 10000.0f);
	//int16_t v2 = (int16_t)(q[2] * 10000.0f);
	//int16_t v3 = (int16_t)(q[3] * 10000.0f);
	//
	//// b. °´С¶˄£ʽ(µֽ͗ڔڇ°)̮³䊽¾ݍ
	//tx_buffer[4] = (uint8_t)(v0 & 0xFF);  // V0 low byte
	//tx_buffer[5] = (uint8_t)(v0 >> 8);	  // V0 high byte
	//tx_buffer[6] = (uint8_t)(v1 & 0xFF);  // V1 low byte
	//tx_buffer[7] = (uint8_t)(v1 >> 8);	  // V1 high byte
	//tx_buffer[8] = (uint8_t)(v2 & 0xFF);  // V2 low byte
	//tx_buffer[9] = (uint8_t)(v2 >> 8);	  // V2 high byte
	//tx_buffer[10] = (uint8_t)(v3 & 0xFF); // V3 low byte
	//tx_buffer[11] = (uint8_t)(v3 >> 8);	  // V3 high byte
	//
	//// c. ̮³䈚ºϗ´̬
	//tx_buffer[12] = 2; // FUSION_STA [cite: 92]
	//
	//// 4. ¼Ƌ㐣ѩº͍
	//// Уѩ·¶Χ£º´Ӡ0xAA µ½ DATAǸ ½ኸ (¹² 13 ז½ں 4 + 9 = 13)
	//uint8_t sum_check = 0;
	//uint8_t add_check = 0;
	//
	//// я¸񰴕Ր­ҩV7.16 P4ҳµĊ¾Àý´ú«½øАУѩ [cite: 21, 23, 27-35]
	//for (int i = 0; i < (tx_buffer[3] + 4); i++) // tx_buffer[3] + 4 = 9 + 4 = 13
	//{
	//	sum_check += tx_buffer[i];
	//	add_check += sum_check;
	//}
	//
	//tx_buffer[13] = sum_check; // º͐£ѩ
	//tx_buffer[14] = add_check; // ¸½¼Ӑ£ѩ
  //BLE_SendArray(tx_buffer, 15);
				
	}
	
}

void TASK_ChangeMotor(void *p_arg){
	
    PPM_Sem = OSSemCreate(0);
	  PID_Controllers_Init();
    while(1){
		INT8U err;
        // �ȴ��ź���
        OSSemPend(PPM_Sem, 0, &err); 
        // ��ȡ���ݷ���Ȩ

        // ���õ��ת��
		if(err==OS_ERR_NONE){
			
			Flight_Control_UpdateOuterLoop(Rc_Data);
		}
		Flight_Control_UpdateInnerLoop();
    }
	
}
