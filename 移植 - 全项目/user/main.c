//main.c
#include <ucos_ii.h>
#include "Task.h"
#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "OLED.h"
#include "PPM.h"
#include "PWM.h"
#include "Delay.h"
#include "MYIIC.h"
#include "GY86.h"
#include "ATKBLE01.h"
#include "stdio.h"
#include "SEGGER_SYSVIEW.h"
#include "getdata.h"

#define  OS_TRACE_INIT()                             SEGGER_SYSVIEW_Conf()
INT8U *pname;
INT8U err;

OS_STK  TASK_ShowGY86Datastk[APP_CFG_STARTUP_TASK_STK_SIZE];
OS_STK  TASK_ChangeMotorstk[APP_CFG_STARTUP_TASK_STK_SIZE];

OS_EVENT *PPM_Sem;    // ����֪ͨ������������

int main(void){
	//Ӳ����ʼ��
	PPM_Init();
	PWM_Init();
	MyIIC_Init();
   MPU6050_Init();  
	HMC5883L_Init();
	BLE_Init();
		
	OS_TRACE_INIT(); //systemview��ʼ��
	OS_TRACE_START(); //��ʼ��¼
	OSInit();
	
	
	//�������
    PWM_SetCompareAll(1000);
	Delay_s(1);
	
	//���񴴽�
	OSTaskCreate(TASK_ChangeMotor,(void*)0,(OS_STK*)&TASK_ChangeMotorstk[APP_CFG_STARTUP_TASK_STK_SIZE-1], 5);
	OSTaskCreate(TASK_ShowGY86Data,(void*)0,(OS_STK*)&TASK_ShowGY86Datastk[APP_CFG_STARTUP_TASK_STK_SIZE-1],6);
	
	// ����������
	OSTaskNameSet(5, (INT8U *)"Change_task", &err);
	OSTaskNameSet(6, (INT8U *)"Show_task", &err);
	
	//��������������䴫��Systemview
	OSTaskNameGet(5, &pname, &err);
	SEGGER_SYSVIEW_NameResource((U32)OSTCBPrioTbl[5], (const char *)pname);
	OSTaskNameGet(6, &pname, &err);
	SEGGER_SYSVIEW_NameResource((U32)OSTCBPrioTbl[6], (const char *)pname);
	OSStart();

  return 0;
	
}
