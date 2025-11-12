"""
仿真用PID参数配置
可以直接修改这些参数进行调试
"""

from controller_unified import UnifiedAttitudeGains


# 默认PID参数（用于初始测试）
DEFAULT_PID_GAINS = UnifiedAttitudeGains(
    # ========== 外环：角度控制（输入：弧度，输出：弧度/秒）==========
    # Roll轴（横滚）
    roll_angle_kp=2.0,      # 比例：增大使响应更快，但可能震荡
    roll_angle_ki=0.0,      # 积分：消除稳态误差，通常从0开始
    roll_angle_kd=0.5,      # 微分：提供阻尼，减少超调
    
    # Pitch轴（俯仰）
    pitch_angle_kp=2.0,     # 通常与Roll相同
    pitch_angle_ki=0.0,     
    pitch_angle_kd=0.5,
    
    # Yaw轴（偏航）
    yaw_angle_kp=1.5,       # Yaw通常比Roll/Pitch小
    yaw_angle_ki=0.0,
    yaw_angle_kd=0.3,
    
    # ========== 内环：角速度控制（输入：弧度/秒，输出：PWM增量）==========
    # Roll速率
    roll_rate_kp=50.0,      # 速率环P增益，影响响应速度
    roll_rate_ki=0.5,       # 速率环I增益，消除稳态误差
    roll_rate_kd=5.0,       # 速率环D增益，抑制高频噪声
    
    # Pitch速率
    pitch_rate_kp=50.0,     # 通常与Roll相同
    pitch_rate_ki=0.5,
    pitch_rate_kd=5.0,
    
    # Yaw速率
    yaw_rate_kp=80.0,       # Yaw速率通常需要更大增益
    yaw_rate_ki=0.5,
    yaw_rate_kd=8.0,
    
    # ========== 输出限制 ==========
    # 角度环输出限制（弧度/秒）
    angle_roll_output_max=5.0,      # 最大期望角速度
    angle_pitch_output_max=5.0,
    angle_yaw_output_max=3.0,       # Yaw通常限制更小
    
    # 速率环输出限制（PWM增量）
    rate_roll_output_max=400.0,     # 最大PWM变化量
    rate_pitch_output_max=400.0,
    rate_yaw_output_max=300.0,      # Yaw通常限制更小
)


# 激进飞行参数（响应快但可能不稳定）
AGGRESSIVE_PID_GAINS = UnifiedAttitudeGains(
    # 外环
    roll_angle_kp=4.0,
    roll_angle_ki=0.1,
    roll_angle_kd=0.8,
    pitch_angle_kp=4.0,
    pitch_angle_ki=0.1,
    pitch_angle_kd=0.8,
    yaw_angle_kp=3.0,
    yaw_angle_ki=0.05,
    yaw_angle_kd=0.5,
    
    # 内环
    roll_rate_kp=80.0,
    roll_rate_ki=1.0,
    roll_rate_kd=8.0,
    pitch_rate_kp=80.0,
    pitch_rate_ki=1.0,
    pitch_rate_kd=8.0,
    yaw_rate_kp=120.0,
    yaw_rate_ki=1.0,
    yaw_rate_kd=10.0,
    
    # 限制
    angle_roll_output_max=7.0,
    angle_pitch_output_max=7.0,
    angle_yaw_output_max=4.0,
    rate_roll_output_max=500.0,
    rate_pitch_output_max=500.0,
    rate_yaw_output_max=400.0,
)


# 保守飞行参数（稳定但响应慢）
CONSERVATIVE_PID_GAINS = UnifiedAttitudeGains(
    # 外环
    roll_angle_kp=1.5,
    roll_angle_ki=0.0,
    roll_angle_kd=0.3,
    pitch_angle_kp=1.5,
    pitch_angle_ki=0.0,
    pitch_angle_kd=0.3,
    yaw_angle_kp=1.0,
    yaw_angle_ki=0.0,
    yaw_angle_kd=0.2,
    
    # 内环
    roll_rate_kp=30.0,
    roll_rate_ki=0.2,
    roll_rate_kd=3.0,
    pitch_rate_kp=30.0,
    pitch_rate_ki=0.2,
    pitch_rate_kd=3.0,
    yaw_rate_kp=50.0,
    yaw_rate_ki=0.2,
    yaw_rate_kd=5.0,
    
    # 限制
    angle_roll_output_max=3.0,
    angle_pitch_output_max=3.0,
    angle_yaw_output_max=2.0,
    rate_roll_output_max=300.0,
    rate_pitch_output_max=300.0,
    rate_yaw_output_max=200.0,
)


# 自定义参数（在这里调整您的PID）
MY_PID_GAINS = UnifiedAttitudeGains(
    # ===== 角度环（外环）=====
    roll_angle_kp=2.5,      # <-- 在这里调整
    roll_angle_ki=0.05,
    roll_angle_kd=0.6,
    
    pitch_angle_kp=2.5,
    pitch_angle_ki=0.05,
    pitch_angle_kd=0.6,
    
    yaw_angle_kp=1.8,
    yaw_angle_ki=0.02,
    yaw_angle_kd=0.4,
    
    # ===== 速率环（内环）=====
    roll_rate_kp=60.0,      # <-- 在这里调整
    roll_rate_ki=0.8,
    roll_rate_kd=6.0,
    
    pitch_rate_kp=60.0,
    pitch_rate_ki=0.8,
    pitch_rate_kd=6.0,
    
    yaw_rate_kp=90.0,
    yaw_rate_ki=0.6,
    yaw_rate_kd=9.0,
    
    # ===== 输出限制 =====
    angle_roll_output_max=5.0,
    angle_pitch_output_max=5.0,
    angle_yaw_output_max=3.0,
    
    rate_roll_output_max=400.0,
    rate_pitch_output_max=400.0,
    rate_yaw_output_max=300.0,
)


def print_pid_for_keil(gains: UnifiedAttitudeGains, name: str = ""):
    """打印可复制到Keil工程的PID参数"""
    if name:
        print(f"\n// {name}")
    print("// ========== 外环PID参数 ==========")
    print(f"#define ANGLE_ROLL_KP           {gains.roll_angle_kp:.1f}f")
    print(f"#define ANGLE_ROLL_KI           {gains.roll_angle_ki:.1f}f")
    print(f"#define ANGLE_ROLL_KD           {gains.roll_angle_kd:.1f}f")
    print(f"#define ANGLE_PITCH_KP          {gains.pitch_angle_kp:.1f}f")
    print(f"#define ANGLE_PITCH_KI          {gains.pitch_angle_ki:.1f}f")
    print(f"#define ANGLE_PITCH_KD          {gains.pitch_angle_kd:.1f}f")
    print(f"#define ANGLE_YAW_KP            {gains.yaw_angle_kp:.1f}f")
    print(f"#define ANGLE_YAW_KI            {gains.yaw_angle_ki:.1f}f")
    print(f"#define ANGLE_YAW_KD            {gains.yaw_angle_kd:.1f}f")
    
    print("\n// ========== 内环PID参数 ==========")
    print(f"#define RATE_ROLL_KP            {gains.roll_rate_kp:.1f}f")
    print(f"#define RATE_ROLL_KI            {gains.roll_rate_ki:.1f}f")
    print(f"#define RATE_ROLL_KD            {gains.roll_rate_kd:.1f}f")
    print(f"#define RATE_PITCH_KP           {gains.pitch_rate_kp:.1f}f")
    print(f"#define RATE_PITCH_KI           {gains.pitch_rate_ki:.1f}f")
    print(f"#define RATE_PITCH_KD           {gains.pitch_rate_kd:.1f}f")
    print(f"#define RATE_YAW_KP             {gains.yaw_rate_kp:.1f}f")
    print(f"#define RATE_YAW_KI             {gains.yaw_rate_ki:.1f}f")
    print(f"#define RATE_YAW_KD             {gains.yaw_rate_kd:.1f}f")
