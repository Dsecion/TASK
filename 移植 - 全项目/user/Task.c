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
       
        int16_t q_roll = (int16_t)(roll*100.0f);
        int16_t q_pitch = (int16_t)(pitch*100.0f);
        int16_t q_yaw = (int16_t)(yaw*100.0f);
       
        int8_t frame_buffer[13];  // æ”¹ä¸ºuint8_t
        uint8_t sum_check = 0, add_check = 0;
        uint8_t data_len = 7;
     
        // ä¿®æ­£å¸§ç»“æ„é¡ºåº
        frame_buffer[0] = 0xAA;     // å¸§å¤´
        frame_buffer[1] = 0xFF;     // ç›®æ ‡åœ°å€ (å¹¿æ’­)
        frame_buffer[2] = 0x03;     // æ•°æ®ID
        frame_buffer[3] = data_len; // æ•°æ®é•¿åº¦
     
        // æ•°æ®éƒ¨åˆ†ä¿æŒä¸å˜
       frame_buffer[4] = q_roll & 0xff;     // å¸§å¤´
        frame_buffer[5] = (q_roll>>8)&0xff;     // ç›®æ ‡åœ°å€ (å¹¿æ’­)
        frame_buffer[6] = q_pitch & 0xff;     // æ•°æ®ID
        frame_buffer[7] = (q_pitch>>8)&0xff;
		frame_buffer[8] = q_yaw& 0xff;     // æ•°æ®ID
        frame_buffer[9] = (q_yaw>>8)&0xff;
		 frame_buffer[10] = 2;
		// FUSION_STAæ”¹ä¸º2
     
        // ä¿®æ­£æ ¡éªŒå’Œè®¡ç®—
        for(int i = 0; i < 4 + data_len; i++) { // i=0åˆ°10
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
