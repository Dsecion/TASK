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
       // int16_t q_roll = (int16_t)(roll*100.0f);
       // int16_t q_pitch = (int16_t)(pitch*100.0f);
       // int16_t q_yaw = (int16_t)(yaw*100.0f);
 //
	//	
 //       int8_t frame_buffer[13];  // 鏀逛负uint8_t
 //       uint8_t sum_check = 0, add_check = 0;
 //       uint8_t data_len = 7;
 //    
 //       // 淇甯х粨鏋勯『搴�
 //       frame_buffer[0] = 0xAA;     // 甯уご
 //       frame_buffer[1] = 0xFF;     // 鐩爣鍦板潃 (骞挎挱)
 //       frame_buffer[2] = 0x03;     // 鏁版嵁ID
 //       frame_buffer[3] = data_len; // 鏁版嵁闀垮害
 //    
 //       // 鏁版嵁閮ㄥ垎淇濇寔涓嶅彉
 //      frame_buffer[4] = q_roll & 0xff;     // 甯уご
 //       frame_buffer[5] = (q_roll>>8)&0xff;     // 鐩爣鍦板潃 (骞挎挱)
 //       frame_buffer[6] = q_pitch & 0xff;     // 鏁版嵁ID
 //       frame_buffer[7] = (q_pitch>>8)&0xff;
	//	frame_buffer[8] = q_yaw& 0xff;     // 鏁版嵁ID
 //       frame_buffer[9] = (q_yaw>>8)&0xff;
	//	 frame_buffer[10] = 2;
	//	// FUSION_STA鏀逛负2
 //    
 //       // 淇鏍￠獙鍜岃绠�
 //       for(int i = 0; i < 4 + data_len; i++) { // i=0鍒�10
 //           sum_check += frame_buffer[i];
 //           add_check += sum_check;
 //       }
 //       frame_buffer[11] = sum_check;
 //       frame_buffer[12] = add_check;
 //      
 //      BLE_SendArray(frame_buffer, 13);
		 //Delay_ms(10);
  uint8_t tx_buffer[15];
  tx_buffer[0] = 0xAA;
	// 2. ̮³䖡ͷº̓üÁ	tx_buffer[0] = 0xAA; // ֡ͷ [cite: 17]
	tx_buffer[1] = 0xFF; // Ŀ±굘ַ (0xFF Ϊ¹㲥) [cite: 90, 426]
	tx_buffer[2] = 0x04; // ¹¦Ĝ« (·ɿؗˌ¬:˄Ԫʽ¸񊽩
	tx_buffer[3] = 9;	 // ʽ¾ݳ¤¶Ƞ(V0,V1,V2,V3,FUSION_STA)

	// 3. ̮³䊽¾݄ڈݠ(DATAǸ)

	// a. ½« float ת»»Ϊ int16_t (°´ЭҩÀ©´󱰰00±¶)
	int16_t v0 = (int16_t)(q[0] * 10000.0f);
	int16_t v1 = (int16_t)(q[1] * 10000.0f);
	int16_t v2 = (int16_t)(q[2] * 10000.0f);
	int16_t v3 = (int16_t)(q[3] * 10000.0f);

	// b. °´С¶˄£ʽ(µֽ͗ڔڇ°)̮³䊽¾ݍ
	tx_buffer[4] = (uint8_t)(v0 & 0xFF);  // V0 low byte
	tx_buffer[5] = (uint8_t)(v0 >> 8);	  // V0 high byte
	tx_buffer[6] = (uint8_t)(v1 & 0xFF);  // V1 low byte
	tx_buffer[7] = (uint8_t)(v1 >> 8);	  // V1 high byte
	tx_buffer[8] = (uint8_t)(v2 & 0xFF);  // V2 low byte
	tx_buffer[9] = (uint8_t)(v2 >> 8);	  // V2 high byte
	tx_buffer[10] = (uint8_t)(v3 & 0xFF); // V3 low byte
	tx_buffer[11] = (uint8_t)(v3 >> 8);	  // V3 high byte

	// c. ̮³䈚ºϗ´̬
	tx_buffer[12] = 1; // FUSION_STA [cite: 92]

	// 4. ¼Ƌ㐣ѩº͍
	// Уѩ·¶Χ£º´Ӡ0xAA µ½ DATAǸ ½ኸ (¹² 13 ז½ں 4 + 9 = 13)
	uint8_t sum_check = 0;
	uint8_t add_check = 0;

	// я¸񰴕Ր­ҩV7.16 P4ҳµĊ¾Àý´ú«½øАУѩ [cite: 21, 23, 27-35]
	for (int i = 0; i < (tx_buffer[3] + 4); i++) // tx_buffer[3] + 4 = 9 + 4 = 13
	{
		sum_check += tx_buffer[i];
		add_check += sum_check;
	}

	// 5. ̮³䐣ѩז½ڍ
	tx_buffer[13] = sum_check; // º͐£ѩ
	tx_buffer[14] = add_check; // ¸½¼Ӑ£ѩ
 // BLE_SendArray(tx_buffer, 15);
				
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
