#ifndef __PID_H_
#define __PID_H_

#include "stm32f4xx.h"

// PID控制器结构体
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    
    float error;        // 当前误差
    float error_last;   // 上一次误差
    float integral;     // 误差积分
    float derivative;   // 误差微分
    
    float output;       // PID输出
    float output_max;   // 输出限幅最大值
    float output_min;   // 输出限幅最小值
    
    float integral_max; // 积分限幅
} PID_Controller;

// PID控制器实例声明
extern PID_Controller pid_roll_angle;
extern PID_Controller pid_pitch_angle;
extern PID_Controller pid_yaw_angle;
extern PID_Controller pid_roll_rate;
extern PID_Controller pid_pitch_rate;
extern PID_Controller pid_yaw_rate;

// PID控制器初始化
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, 
              float out_max, float out_min, float integral_max);

// PID计算
float PID_Calculate(PID_Controller *pid, float setpoint, float measurement);

// 重置PID控制器
void PID_Reset(PID_Controller *pid);

// 初始化所有PID控制器
void PID_Controllers_Init(void);

#endif

