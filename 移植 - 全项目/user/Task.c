//Task.c文件，设置发送数据优先级最高
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

// 四元数数据存储
float quaternion_w, quaternion_x, quaternion_y, quaternion_z; 


void TIM2_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr = 0u;
    OSIntEnter();  // 进入中断

    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
    {
        uint32_t Rc_Val = TIM_GetCapture1(TIM2);
        
        //这里需要保护共享数据
        OS_ENTER_CRITICAL();
        if(Rc_Val > 2050){
            Cal = 0;
        }
        else if(Cal < 8){
            Rc_Data[Cal] = Rc_Val;
            Cal++;
            
            // 当8通道数据收齐时发布信号量
            if(Cal == 8){
                OSSemPost(PPM_Sem);  // 通知任务
				Cal=0;//重置计数器
            }
        }
        OS_EXIT_CRITICAL();
        
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
    }
    
    OSIntExit();  // 退出中断
}




void TASK_ShowGY86Data(void *p_arg){

	while(1){
		
        // 获取原始传感器数据
        GY86_GetData(&x, &y, &z, &AX, &AY, &AZ, &GX, &GY, &GZ);
        
        // 执行四元数融合算法
        Getdata();
        
        // 更新四元数数据
        quaternion_w = q[0];
        quaternion_x = q[1];
        quaternion_y = q[2];
        quaternion_z = q[3];
        
        // 按照通信帧格式发送四元数数据
        uint8_t frame_buffer[15];  // 完整帧缓冲区
        uint8_t frame_index = 0;
       
        // 1. 帧头 (HEAD) - 0xAB
        frame_buffer[frame_index++] = 0xAA;
        
        // 2. 源地址 (S_ADDR) - 设备ID，这里设为0x01
        frame_buffer[frame_index++] = 0xFF;
        
        // 3. 目标地址 (D_ADDR) - 接收设备ID，这里设为0x02
        frame_buffer[frame_index++] = 0x04;
      
        // 5. 数据长度 (LEN) - 小端序，16字节
        frame_buffer[frame_index++] = 0x09;        // 低字节
     
        // 6. 数据内容 (DATA) - 四元数数据（16字节）
        memcpy(&frame_buffer[frame_index], &quaternion_w, 4);
        frame_index += 2;
        memcpy(&frame_buffer[frame_index], &quaternion_x, 4);
        frame_index += 2;
        memcpy(&frame_buffer[frame_index], &quaternion_y, 4);
        frame_index += 2;
        memcpy(&frame_buffer[frame_index], &quaternion_z, 4);
        frame_index += 2;
        frame_buffer[frame_index++] = (uint8_t)(0); 
        // 7. 和校验 (SUM CHECK) 和 8. 附加校验 (ADD CHECK)
        // 按照图片中的算法：SUM CHECK是累加校验，ADD CHECK是对SUM CHECK中间值的累加
        uint8_t sum_check, add_check;
        CalculateChecksums(frame_buffer, frame_index, &sum_check, &add_check);
        
        frame_buffer[frame_index++] = sum_check;
        frame_buffer[frame_index++] = add_check;
        
        // 发送完整帧
        BLE_SendArray(frame_buffer, 15);
        
        // 任务延时，控制数据发送频率
        OSTimeDly(10); // 延时10个系统时钟周期，约100ms（假设系统时钟为100Hz）
	}
}
void TASK_ChangeMotor(void *p_arg){
	
    PPM_Sem = OSSemCreate(0);
    while(1){
		INT8U err;
        // 等待信号量
        OSSemPend(PPM_Sem, 0, &err); 
        // 获取数据访问权
        uint16_t throttle = Rc_Data[1];  //通道1控制油门
        // 设置电机转速
		if(err==OS_ERR_NONE){
			
			PWM_SetCompareAll(throttle);
		}
    }
	
}
