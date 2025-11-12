"""
PID参数导出工具
将调试好的PID参数导出为Keil工程格式
"""

from controller_unified import UnifiedAttitudeGains


def generate_keil_config(gains: UnifiedAttitudeGains, output_path: str = None) -> str:
    """
    生成Keil工程的PID配置代码
    
    Args:
        gains: 统一的增益对象
        output_path: 输出文件路径（可选）
        
    Returns:
        生成的C代码字符串
    """
    config_str = """#ifndef __PID_CONFIG_H_
#define __PID_CONFIG_H_

/******************************************************************************
 * 外环PID参数（输出：目标角速度 rad/s）
 * 由Python仿真优化生成
 ******************************************************************************/

// Roll轴（横滚）角度环
#define ANGLE_ROLL_KP           {:.1f}f    // 比例系数
#define ANGLE_ROLL_KI           {:.1f}f    // 积分系数
#define ANGLE_ROLL_KD           {:.1f}f    // 微分系数
#define ANGLE_ROLL_OUTPUT_MAX   {:.1f}f    // 输出最大值（rad/s）
#define ANGLE_ROLL_OUTPUT_MIN  -{:.1f}f    // 输出最小值（rad/s）
#define ANGLE_ROLL_INTEGRAL_MAX 10.0f      // 积分限幅

// Pitch轴（俯仰）角度环
#define ANGLE_PITCH_KP          {:.1f}f
#define ANGLE_PITCH_KI          {:.1f}f
#define ANGLE_PITCH_KD          {:.1f}f
#define ANGLE_PITCH_OUTPUT_MAX  {:.1f}f
#define ANGLE_PITCH_OUTPUT_MIN -{:.1f}f
#define ANGLE_PITCH_INTEGRAL_MAX 10.0f

// Yaw轴（偏航）角度环
#define ANGLE_YAW_KP            {:.1f}f
#define ANGLE_YAW_KI            {:.1f}f
#define ANGLE_YAW_KD            {:.1f}f
#define ANGLE_YAW_OUTPUT_MAX    {:.1f}f
#define ANGLE_YAW_OUTPUT_MIN   -{:.1f}f
#define ANGLE_YAW_INTEGRAL_MAX  5.0f

/******************************************************************************
 * 内环PID参数（输出：PWM增量）
 ******************************************************************************/

// Roll轴（横滚）角速度环
#define RATE_ROLL_KP            {:.1f}f    // 比例系数
#define RATE_ROLL_KI            {:.1f}f    // 积分系数
#define RATE_ROLL_KD            {:.1f}f    // 微分系数
#define RATE_ROLL_OUTPUT_MAX    {:.1f}f    // 输出最大值（PWM增量）
#define RATE_ROLL_OUTPUT_MIN   -{:.1f}f    // 输出最小值（PWM增量）
#define RATE_ROLL_INTEGRAL_MAX  50.0f      // 积分限幅

// Pitch轴（俯仰）角速度环
#define RATE_PITCH_KP           {:.1f}f
#define RATE_PITCH_KI           {:.1f}f
#define RATE_PITCH_KD           {:.1f}f
#define RATE_PITCH_OUTPUT_MAX   {:.1f}f
#define RATE_PITCH_OUTPUT_MIN  -{:.1f}f
#define RATE_PITCH_INTEGRAL_MAX 50.0f

// Yaw轴（偏航）角速度环
#define RATE_YAW_KP             {:.1f}f
#define RATE_YAW_KI             {:.1f}f
#define RATE_YAW_KD             {:.1f}f
#define RATE_YAW_OUTPUT_MAX     {:.1f}f
#define RATE_YAW_OUTPUT_MIN    -{:.1f}f
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
""".format(
        # Roll角度环
        gains.roll_angle_kp, gains.roll_angle_ki, gains.roll_angle_kd,
        gains.angle_roll_output_max, gains.angle_roll_output_max,
        # Pitch角度环
        gains.pitch_angle_kp, gains.pitch_angle_ki, gains.pitch_angle_kd,
        gains.angle_pitch_output_max, gains.angle_pitch_output_max,
        # Yaw角度环
        gains.yaw_angle_kp, gains.yaw_angle_ki, gains.yaw_angle_kd,
        gains.angle_yaw_output_max, gains.angle_yaw_output_max,
        # Roll角速度环
        gains.roll_rate_kp, gains.roll_rate_ki, gains.roll_rate_kd,
        gains.rate_roll_output_max, gains.rate_roll_output_max,
        # Pitch角速度环
        gains.pitch_rate_kp, gains.pitch_rate_ki, gains.pitch_rate_kd,
        gains.rate_pitch_output_max, gains.rate_pitch_output_max,
        # Yaw角速度环
        gains.yaw_rate_kp, gains.yaw_rate_ki, gains.yaw_rate_kd,
        gains.rate_yaw_output_max, gains.rate_yaw_output_max,
    )
    
    if output_path:
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(config_str)
        print(f"已生成Keil配置文件: {output_path}")
    
    return config_str


def print_simple_pid_params(gains: UnifiedAttitudeGains, name: str = ""):
    """打印简化的PID参数（只包含最重要的参数）"""
    if name:
        print(f"\n=== {name} ===")
    
    print("\n外环（角度控制）:")
    print(f"  Roll:  P={gains.roll_angle_kp:.1f}  I={gains.roll_angle_ki:.2f}  D={gains.roll_angle_kd:.1f}")
    print(f"  Pitch: P={gains.pitch_angle_kp:.1f}  I={gains.pitch_angle_ki:.2f}  D={gains.pitch_angle_kd:.1f}")
    print(f"  Yaw:   P={gains.yaw_angle_kp:.1f}  I={gains.yaw_angle_ki:.2f}  D={gains.yaw_angle_kd:.1f}")
    
    print("\n内环（角速度控制）:")
    print(f"  Roll:  P={gains.roll_rate_kp:.0f}  I={gains.roll_rate_ki:.1f}  D={gains.roll_rate_kd:.1f}")
    print(f"  Pitch: P={gains.pitch_rate_kp:.0f}  I={gains.pitch_rate_ki:.1f}  D={gains.pitch_rate_kd:.1f}")
    print(f"  Yaw:   P={gains.yaw_rate_kp:.0f}  I={gains.yaw_rate_ki:.1f}  D={gains.yaw_rate_kd:.1f}")


def main():
    """演示如何使用参数导出工具"""
    from pid_params import MY_PID_GAINS, DEFAULT_PID_GAINS, AGGRESSIVE_PID_GAINS, CONSERVATIVE_PID_GAINS
    
    print("PID参数导出工具")
    print("=" * 50)
    
    print("\n可用的预设参数：")
    print_simple_pid_params(DEFAULT_PID_GAINS, "默认参数")
    print_simple_pid_params(CONSERVATIVE_PID_GAINS, "保守参数")
    print_simple_pid_params(AGGRESSIVE_PID_GAINS, "激进参数")
    print_simple_pid_params(MY_PID_GAINS, "自定义参数")
    
    print("\n" + "=" * 50)
    print("\n将自定义参数导出为Keil格式：")
    print("\n复制以下内容到 PID_Config.h:")
    print("-" * 50)
    
    # 只打印最关键的定义
    gains = MY_PID_GAINS
    print("// ===== 外环PID参数 =====")
    print(f"#define ANGLE_ROLL_KP           {gains.roll_angle_kp:.1f}f")
    print(f"#define ANGLE_ROLL_KI           {gains.roll_angle_ki:.1f}f")
    print(f"#define ANGLE_ROLL_KD           {gains.roll_angle_kd:.1f}f")
    print(f"#define ANGLE_PITCH_KP          {gains.pitch_angle_kp:.1f}f")
    print(f"#define ANGLE_PITCH_KI          {gains.pitch_angle_ki:.1f}f")
    print(f"#define ANGLE_PITCH_KD          {gains.pitch_angle_kd:.1f}f")
    print(f"#define ANGLE_YAW_KP            {gains.yaw_angle_kp:.1f}f")
    print(f"#define ANGLE_YAW_KI            {gains.yaw_angle_ki:.1f}f")
    print(f"#define ANGLE_YAW_KD            {gains.yaw_angle_kd:.1f}f")
    
    print("\n// ===== 内环PID参数 =====")
    print(f"#define RATE_ROLL_KP            {gains.roll_rate_kp:.1f}f")
    print(f"#define RATE_ROLL_KI            {gains.roll_rate_ki:.1f}f")
    print(f"#define RATE_ROLL_KD            {gains.roll_rate_kd:.1f}f")
    print(f"#define RATE_PITCH_KP           {gains.pitch_rate_kp:.1f}f")
    print(f"#define RATE_PITCH_KI           {gains.pitch_rate_ki:.1f}f")
    print(f"#define RATE_PITCH_KD           {gains.pitch_rate_kd:.1f}f")
    print(f"#define RATE_YAW_KP             {gains.yaw_rate_kp:.1f}f")
    print(f"#define RATE_YAW_KI             {gains.yaw_rate_ki:.1f}f")
    print(f"#define RATE_YAW_KD             {gains.yaw_rate_kd:.1f}f")
    
    print("\n" + "-" * 50)
    print("\n提示：")
    print("1. 在 pid_params.py 中修改 MY_PID_GAINS 来调整参数")
    print("2. 运行仿真测试参数效果")
    print("3. 将优化后的参数复制到Keil工程")
    print("4. 建议先在地面测试，确保安全")


if __name__ == "__main__":
    main()