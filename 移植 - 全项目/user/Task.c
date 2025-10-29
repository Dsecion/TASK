//Task.cÎÄ¼ş£¬ÉèÖÃ·¢ËÍÊı¾İÓÅÏÈ¼¶×î¸ß
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



// ËÄÔªÊıÊı¾İ´æ´¢
int16_t quaternion_w, quaternion_x, quaternion_y, quaternion_z; 


void TIM2_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr = 0u;
    OSIntEnter();  // ½øÈëÖĞ¶Ï

    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
    {
        uint32_t Rc_Val = TIM_GetCapture1(TIM2);
        
        //ÕâÀïĞèÒª±£»¤¹²ÏíÊı¾İ
        OS_ENTER_CRITICAL();
        if(Rc_Val > 2050){
            Cal = 0;
        }
        else if(Cal < 8){
            Rc_Data[Cal] = Rc_Val;
            Cal++;
            
            // µ±8Í¨µÀÊı¾İÊÕÆëÊ±·¢²¼ĞÅºÅÁ¿
            if(Cal == 8){
                OSSemPost(PPM_Sem);  // Í¨ÖªÈÎÎñ
				Cal=0;//ÖØÖÃ¼ÆÊıÆ÷
            }
        }
        OS_EXIT_CRITICAL();
        
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
    }
    
    OSIntExit();  // ÍË³öÖĞ¶Ï
}




void TASK_ShowGY86Data(void *p_arg){
    
    while(1){
		Getdata();
    
  uint8_t tx_buffer[15];
  tx_buffer[0] = 0xAA;

	tx_buffer[1] = 0xFF; // Ä¿Â±êµ˜Ö· (0xFF ÎªÂ¹ã²¥) [cite: 90, 426]
	tx_buffer[2] = 0x04; // Â¹Â¦ÄœÂ« (Â·É¿Ø—ËŒÂ¬:Ë„ÔªÊ½Â¸ñŠ½©
	tx_buffer[3] = 9;	 // Ê½Â¾İ³Â¤Â¶È (V0,V1,V2,V3,FUSION_STA)

	// 3. Ì®Â³äŠ½Â¾İ„Úˆİ (DATAÇ¸)

	// a. Â½Â« float ×ªÂ»Â»Îª int16_t (Â°Â´Ğ­Ò©Ã€Â©Â´ó±°°00Â±Â¶)
	int16_t v0 = (int16_t)(q[0] * 10000.0f);
	int16_t v1 = (int16_t)(q[1] * 10000.0f);
	int16_t v2 = (int16_t)(q[2] * 10000.0f);
	int16_t v3 = (int16_t)(q[3] * 10000.0f);

	// b. Â°Â´Ğ¡Â¶Ë„Â£Ê½(ÂµÍ—Ö½Ú”Ú‡Â°)Ì®Â³äŠ½Â¾İ
	tx_buffer[4] = (uint8_t)(v0 & 0xFF);  // V0 low byte
	tx_buffer[5] = (uint8_t)(v0 >> 8);	  // V0 high byte
	tx_buffer[6] = (uint8_t)(v1 & 0xFF);  // V1 low byte
	tx_buffer[7] = (uint8_t)(v1 >> 8);	  // V1 high byte
	tx_buffer[8] = (uint8_t)(v2 & 0xFF);  // V2 low byte
	tx_buffer[9] = (uint8_t)(v2 >> 8);	  // V2 high byte
	tx_buffer[10] = (uint8_t)(v3 & 0xFF); // V3 low byte
	tx_buffer[11] = (uint8_t)(v3 >> 8);	  // V3 high byte

	// c. Ì®Â³äˆšÂºÏ—Â´Ì¬
	tx_buffer[12] = 2; // FUSION_STA [cite: 92]

	// 4. Â¼Æ‹ã£Ñ©ÂºÍ
	// Ğ£Ñ©Â·Â¶Î§Â£ÂºÂ´Ó 0xAA ÂµÂ½ DATAÇ¸ Â½áŠ¸ (Â¹Â² 13 ×–Â½Úº 4 + 9 = 13)
	uint8_t sum_check = 0;
	uint8_t add_check = 0;

	// ÑÂ¸ñ°´•ÕÂ­Ò©V7.16 P4Ò³ÂµÄŠÂ¾Ã€Ã½Â´ÃºÂ«Â½Ã¸ĞĞ£Ñ© [cite: 21, 23, 27-35]
	for (int i = 0; i < (tx_buffer[3] + 4); i++) // tx_buffer[3] + 4 = 9 + 4 = 13
	{
		sum_check += tx_buffer[i];
		add_check += sum_check;
	}

	tx_buffer[13] = sum_check; // ÂºÍÂ£Ñ©
	tx_buffer[14] = add_check; // Â¸Â½Â¼ÓÂ£Ñ©
  BLE_SendArray(tx_buffer, 15);
				
	}
	
}

void TASK_ChangeMotor(void *p_arg){
	
    PPM_Sem = OSSemCreate(0);
    while(1){
		INT8U err;
        // µÈ´ıĞÅºÅÁ¿
        OSSemPend(PPM_Sem, 0, &err); 
        // »ñÈ¡Êı¾İ·ÃÎÊÈ¨
        uint16_t throttle = Rc_Data[1];  //Í¨µÀ1¿ØÖÆÓÍÃÅ
        // ÉèÖÃµç»ú×ªËÙ
		if(err==OS_ERR_NONE){
			
			PWM_SetCompareAll(throttle);
		}
    }
	
}
