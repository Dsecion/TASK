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
#include <stdlib.h>

// 外部PID控制器引用
extern PID_Controller pid_roll_angle;
extern PID_Controller pid_pitch_angle;
extern PID_Controller pid_yaw_angle;
extern PID_Controller pid_roll_rate;
extern PID_Controller pid_pitch_rate;
extern PID_Controller pid_yaw_rate;

// 控制状态标志
static volatile uint8_t control_enabled = 0;
static volatile uint8_t is_motor_started = 0;
static volatile uint8_t safe_start_passed = 0; // 上电安全检查标志

// 外环与油门设定
static volatile float throttle_command = MOTOR_MIN;
static volatile float roll_angle_target = 0.0f;
static volatile float pitch_angle_target = 0.0f;
static volatile float yaw_rate_target = 0.0f;

// 内环角速度期望
static volatile float roll_rate_target = 0.0f;
static volatile float pitch_rate_target = 0.0f;
static volatile float yaw_rate_target_cmd = 0.0f;

static void Flight_Control_ResetState(void)
{
    PID_Reset(&pid_roll_angle);
    PID_Reset(&pid_pitch_angle);
    PID_Reset(&pid_yaw_angle);
    PID_Reset(&pid_roll_rate);
    PID_Reset(&pid_pitch_rate);
    PID_Reset(&pid_yaw_rate);

    throttle_command = MOTOR_MIN;
    roll_angle_target = 0.0f;
    pitch_angle_target = 0.0f;
    yaw_rate_target = 0.0f;
    roll_rate_target = 0.0f;
    pitch_rate_target = 0.0f;
    yaw_rate_target_cmd = 0.0f;
}

/**
 * @brief  遥控器驱动的外环更新（低频执行）
 * @param  rc_data: 遥控器数据数组[8]
 */
void Flight_Control_UpdateOuterLoop(uint16_t *rc_data)
{
    int16_t roll_stick, pitch_stick, yaw_stick;
    float throttle_input;

    throttle_input = (float)rc_data[1];
    
    // 提前读取Yaw杆量用于解锁/上锁判断
    yaw_stick = rc_data[0] - RC_CENTER;

    // --- 上电安全检查 ---
    // 防止上电时油门处于高位导致电机直接旋转
    if (!safe_start_passed) {
        if (throttle_input < THROTTLE_MIN) {
            safe_start_passed = 1; // 只有检测到一次油门归零，才允许后续操作
        } else {
            // 油门未归零，强制锁定
            control_enabled = 0;
            PWM_SetCompareAll(MOTOR_MIN);
            return;
        }
    }

    // 状态机逻辑：
    // 1. 如果油门推起 (> THROTTLE_MIN)，标记电机已启动
    // 2. 如果油门拉低 (< THROTTLE_MIN)：
    //    a. 如果Yaw杆打到最左边 (<-400)，则执行上锁（停止电机）
    //    b. 如果未上锁且电机已启动，则保持控制（怠速模式/Air Mode），允许PID在低油门下介入
    //    c. 如果电机未启动，则保持锁定状态

    if (throttle_input > THROTTLE_MIN) {
        is_motor_started = 1;
    }
    
    if (throttle_input < THROTTLE_MIN) {
        // 上锁指令：油门低位 + Yaw向左打死
        if (yaw_stick < -400) { 
            is_motor_started = 0;
        }

        // 如果电机未启动，则执行复位和停机
        if (!is_motor_started) {
            control_enabled = 0;
            Flight_Control_ResetState();
            PWM_SetCompareAll(MOTOR_MIN);
            return;
        }
        // 如果电机已启动，不返回，继续执行后续PID计算（Air Mode）
    }

    control_enabled = 1;

    // 读取并处理遥控器通道值
    roll_stick = rc_data[3] - RC_CENTER;
    pitch_stick = rc_data[2] - RC_CENTER;
    // yaw_stick 已在上面读取

    // 死区处理
    if (abs(roll_stick) < RC_DEADBAND) roll_stick = 0;
    if (abs(pitch_stick) < RC_DEADBAND) pitch_stick = 0;
    if (abs(yaw_stick) < RC_DEADBAND) yaw_stick = 0;

    throttle_command = throttle_input;

    // 将遥控器输入转换为目标角度（弧度）
    roll_angle_target = (roll_stick / RC_RANGE) * ANGLE_MAX_RAD;
    pitch_angle_target = (pitch_stick / RC_RANGE) * ANGLE_MAX_RAD;
    yaw_rate_target = (yaw_stick / RC_RANGE) * YAW_RATE_MAX;

    // 使用 Madgwick 姿态解算得到的当前姿态
    roll_rate_target = PID_Calculate(&pid_roll_angle, roll_angle_target, roll);
    pitch_rate_target = PID_Calculate(&pid_pitch_angle, pitch_angle_target, pitch);

    // Yaw 轴保持速率模式，外环直接给出角速度
    yaw_rate_target_cmd = yaw_rate_target;

    // 限幅油门，避免超出物理范围
    if (throttle_command > MOTOR_MAX) {
        throttle_command = MOTOR_MAX;
    }
    if (throttle_command < MOTOR_MIN) {
        throttle_command = MOTOR_MIN;
    }
}

/**
 * @brief  内环角速度控制更新（高频执行）
 * @note   需在传感器更新任务中周期调用
 */
void Flight_Control_UpdateInnerLoop(void)
{
    float roll_output, pitch_output, yaw_output;

    if (!control_enabled) {
        PWM_SetCompareAll(MOTOR_MIN);
        return;
    }

    // 根据最新角速度反馈执行内环 PID
    roll_output = PID_Calculate(&pid_roll_rate, roll_rate_target, gyro_roll);
    pitch_output = PID_Calculate(&pid_pitch_rate, pitch_rate_target, gyro_pitch);
    yaw_output = PID_Calculate(&pid_yaw_rate, yaw_rate_target_cmd, gyro_yaw);

    PWM_Motor_Mixing(throttle_command, roll_output, pitch_output, yaw_output);
}
