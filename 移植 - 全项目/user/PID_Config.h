#ifndef __PID_CONFIG_H_
#define __PID_CONFIG_H_

/******************************************************************************
 * 外环PID参数（输出：目标角速度 rad/s）
 ******************************************************************************/

// Roll轴（横滚）角度环
#define ANGLE_ROLL_KP           2.0f    // 比例系数
#define ANGLE_ROLL_KI           0.0f    // 积分系数
#define ANGLE_ROLL_KD           0.5f    // 微分系数
#define ANGLE_ROLL_OUTPUT_MAX   5.0f    // 输出最大值（rad/s）
#define ANGLE_ROLL_OUTPUT_MIN  -5.0f    // 输出最小值（rad/s）
#define ANGLE_ROLL_INTEGRAL_MAX 10.0f   // 积分限幅

// Pitch轴（俯仰）角度环
#define ANGLE_PITCH_KP          2.0f
#define ANGLE_PITCH_KI          0.0f
#define ANGLE_PITCH_KD          0.5f
#define ANGLE_PITCH_OUTPUT_MAX  5.0f
#define ANGLE_PITCH_OUTPUT_MIN -5.0f
#define ANGLE_PITCH_INTEGRAL_MAX 10.0f

// Yaw轴（偏航）角度环
#define ANGLE_YAW_KP            1.5f
#define ANGLE_YAW_KI            0.0f
#define ANGLE_YAW_KD            0.3f
#define ANGLE_YAW_OUTPUT_MAX    3.0f
#define ANGLE_YAW_OUTPUT_MIN   -3.0f
#define ANGLE_YAW_INTEGRAL_MAX  5.0f

/******************************************************************************
 * 内环PID参数（输出：PWM增量）
 ******************************************************************************/

// Roll轴（横滚）角速度环
#define RATE_ROLL_KP            50.0f   // 比例系数
#define RATE_ROLL_KI            0.5f    // 积分系数
#define RATE_ROLL_KD            5.0f    // 微分系数
#define RATE_ROLL_OUTPUT_MAX    400.0f  // 输出最大值（PWM增量）
#define RATE_ROLL_OUTPUT_MIN   -400.0f  // 输出最小值（PWM增量）
#define RATE_ROLL_INTEGRAL_MAX  50.0f   // 积分限幅

// Pitch轴（俯仰）角速度环
#define RATE_PITCH_KP           50.0f
#define RATE_PITCH_KI           0.5f
#define RATE_PITCH_KD           5.0f
#define RATE_PITCH_OUTPUT_MAX   400.0f
#define RATE_PITCH_OUTPUT_MIN  -400.0f
#define RATE_PITCH_INTEGRAL_MAX 50.0f

// Yaw轴（偏航）角速度环
#define RATE_YAW_KP             80.0f
#define RATE_YAW_KI             0.5f
#define RATE_YAW_KD             8.0f
#define RATE_YAW_OUTPUT_MAX     300.0f
#define RATE_YAW_OUTPUT_MIN    -300.0f
#define RATE_YAW_INTEGRAL_MAX   50.0f

/******************************************************************************
 * 飞控系统参数配置
 ******************************************************************************/

// 遥控器参数
#define RC_CENTER               1500    // 遥控器中点值（μs）
#define RC_DEADBAND             10      // 遥控器死区（μs）
#define RC_RANGE                500.0f  // 遥控器有效范围（从中点到最大/最小）

// 电机参数
#define MOTOR_MIN               1000    // 电机PWM最小值（μs）
#define MOTOR_MAX               2000    // 电机PWM最大值（μs）
#define THROTTLE_MIN            1100    // 油门激活阈值（低于此值锁定电机）

// 角度限制参数
#define ANGLE_MAX_DEG           30.0f   // 最大倾斜角度（度）
#define ANGLE_MAX_RAD           (ANGLE_MAX_DEG * 3.14159f / 180.0f)  // 转换为弧度

// Yaw轴参数（使用角速度模式）
#define YAW_RATE_MAX            2.0f    // Yaw轴最大角速度（rad/s）

#endif /* __PID_CONFIG_H_ */