/******************************************************************************
 * 文件名：Flight_Control.c
 * 功能：飞行控制核心算法实现
 ******************************************************************************/

#include "Flight_Control.h"
#include "PID_Config.h"
#include "PID.h"
#include "PWM.h"
#include "getdata.h"
#include <math.h>

// 外部PID控制器引用
extern PID_Controller pid_roll_angle;
extern PID_Controller pid_pitch_angle;
extern PID_Controller pid_yaw_angle;
extern PID_Controller pid_roll_rate;
extern PID_Controller pid_pitch_rate;
extern PID_Controller pid_yaw_rate;

// 控制状态标志
static uint8_t control_enabled = 0;

/**
 * @brief  飞行控制主函数
 * @param  rc_data: 遥控器数据数组[8]
 * @note   执行流程：
 *         1. 处理遥控器输入
 *         2. 安全检查
 *         3. 角度外环PID
 *         4. 角速度内环PID
 *         5. 电机混控输出
 */
void Flight_Control(uint16_t *rc_data)
{
    int16_t roll_stick, pitch_stick, yaw_stick;
    float throttle_base;
    float roll_target, pitch_target, yaw_rate_target;
    float roll_rate_target, pitch_rate_target;
    float roll_output, pitch_output, yaw_output;
    
    // 获取油门值
    throttle_base = (float)rc_data[2];
    
    // 安全检查：油门过低时锁定电机
    if (throttle_base < THROTTLE_MIN) {
        PWM_SetCompareAll(MOTOR_MIN);
        
        // 重置所有PID控制器
        PID_Reset(&pid_roll_angle);
        PID_Reset(&pid_pitch_angle);
        PID_Reset(&pid_yaw_angle);
        PID_Reset(&pid_roll_rate);
        PID_Reset(&pid_pitch_rate);
        PID_Reset(&pid_yaw_rate);
        
        control_enabled = 0;
        return;
    }
    
    control_enabled = 1;
    
    // ========== 步骤1：处理遥控器输入 ==========
    // 读取并处理遥控器通道值
    // 通道1: Roll（横滚）
    // 通道2: Pitch（俯仰）
    // 通道3: 油门值（PWM）
    // 通道4: Yaw（偏航）
    roll_stick = rc_data[0] - RC_CENTER;
    pitch_stick = rc_data[1] - RC_CENTER;
    yaw_stick = rc_data[3] - RC_CENTER;
    
    // 死区处理
    if (abs(roll_stick) < RC_DEADBAND) roll_stick = 0;
    if (abs(pitch_stick) < RC_DEADBAND) pitch_stick = 0;
    if (abs(yaw_stick) < RC_DEADBAND) yaw_stick = 0;
    
    // ========== 步骤2：计算目标姿态 ==========
    // 将遥控器输入转换为目标角度（弧度）
    roll_target = (roll_stick / RC_RANGE) * ANGLE_MAX_RAD;
    pitch_target = (pitch_stick / RC_RANGE) * ANGLE_MAX_RAD;
    yaw_rate_target = (yaw_stick / RC_RANGE) * YAW_RATE_MAX;
    
    // ========== 步骤3：角度外环PID控制 ==========
    // 计算期望的角速度
    roll_rate_target = PID_Calculate(&pid_roll_angle, roll_target, roll);
    pitch_rate_target = PID_Calculate(&pid_pitch_angle, pitch_target, pitch);
    
    // ========== 步骤4：角速度内环PID控制 ==========
    // 计算控制输出
    roll_output = PID_Calculate(&pid_roll_rate, roll_rate_target, gyro_roll);
    pitch_output = PID_Calculate(&pid_pitch_rate, pitch_rate_target, gyro_pitch);
    yaw_output = PID_Calculate(&pid_yaw_rate, yaw_rate_target, gyro_yaw);
    
    // ========== 步骤5：电机混控输出 ==========
    PWM_Motor_Mixing(throttle_base, roll_output, pitch_output, yaw_output);
}
