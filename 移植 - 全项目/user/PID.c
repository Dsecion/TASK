#include "PID.h"
#include "PID_Config.h"
#include <string.h>

// PID控制器实例定义
PID_Controller pid_roll_angle;
PID_Controller pid_pitch_angle;
PID_Controller pid_yaw_angle;
PID_Controller pid_roll_rate;
PID_Controller pid_pitch_rate;
PID_Controller pid_yaw_rate;

/**
 * @brief  PID控制器初始化
 * @param  pid: PID控制器指针
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @param  out_max: 输出最大值
 * @param  out_min: 输出最小值
 * @param  integral_max: 积分限幅
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, 
              float out_max, float out_min, float integral_max)
{
    memset(pid, 0, sizeof(PID_Controller));
    
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    
    pid->output_max = out_max;
    pid->output_min = out_min;
    pid->integral_max = integral_max;
}

/**
 * @brief  PID计算
 * @param  pid: PID控制器指针
 * @param  setpoint: 目标值
 * @param  measurement: 测量值
 * @retval PID输出值
 */
float PID_Calculate(PID_Controller *pid, float setpoint, float measurement)
{
    // 计算误差
    pid->error = setpoint - measurement;
    
    // 计算积分(带限幅)
    pid->integral += pid->error;
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }
    
    // 计算微分
    pid->derivative = pid->error - pid->error_last;
    
    // PID计算
    pid->output = pid->Kp * pid->error 
                + pid->Ki * pid->integral 
                + pid->Kd * pid->derivative;
    
    // 输出限幅
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }
    
    // 保存当前误差
    pid->error_last = pid->error;
    
    return pid->output;
}

/**
 * @brief  重置PID控制器
 * @param  pid: PID控制器指针
 */
void PID_Reset(PID_Controller *pid)
{
    pid->error = 0;
    pid->error_last = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;
}

/**
 * @brief  初始化所有PID控制器
 * @note   参数从PID_Config.h读取
 */
void PID_Controllers_Init(void)
{
    // 角度外环PID初始化
    PID_Init(&pid_roll_angle,  
             ANGLE_ROLL_KP, ANGLE_ROLL_KI, ANGLE_ROLL_KD,
             ANGLE_ROLL_OUTPUT_MAX, ANGLE_ROLL_OUTPUT_MIN, ANGLE_ROLL_INTEGRAL_MAX);
    
    PID_Init(&pid_pitch_angle, 
             ANGLE_PITCH_KP, ANGLE_PITCH_KI, ANGLE_PITCH_KD,
             ANGLE_PITCH_OUTPUT_MAX, ANGLE_PITCH_OUTPUT_MIN, ANGLE_PITCH_INTEGRAL_MAX);
    
    PID_Init(&pid_yaw_angle,   
             ANGLE_YAW_KP, ANGLE_YAW_KI, ANGLE_YAW_KD,
             ANGLE_YAW_OUTPUT_MAX, ANGLE_YAW_OUTPUT_MIN, ANGLE_YAW_INTEGRAL_MAX);
    
    // 角速度内环PID初始化
    PID_Init(&pid_roll_rate,  
             RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD,
             RATE_ROLL_OUTPUT_MAX, RATE_ROLL_OUTPUT_MIN, RATE_ROLL_INTEGRAL_MAX);
    
    PID_Init(&pid_pitch_rate, 
             RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD,
             RATE_PITCH_OUTPUT_MAX, RATE_PITCH_OUTPUT_MIN, RATE_PITCH_INTEGRAL_MAX);
    
    PID_Init(&pid_yaw_rate,   
             RATE_YAW_KP, RATE_YAW_KI, RATE_YAW_KD,
             RATE_YAW_OUTPUT_MAX, RATE_YAW_OUTPUT_MIN, RATE_YAW_INTEGRAL_MAX);
}

