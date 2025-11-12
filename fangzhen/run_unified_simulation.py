"""
统一仿真示例
演示如何使用与Keil工程兼容的参数运行仿真
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import List, Dict

# 导入统一的组件
from unified_params import UnifiedQuadParams, DEFAULT_UNIFIED_PARAMS
from controller_unified import UnifiedUavController, UnifiedAttitudeGains
from dynamics_unified import UnifiedQuadrotorDynamics
from mixer_x_type import motor_mixing_to_pwm
from sensors import SensorNoise, NoisyReadings

# 导入PID参数
from pid_params import MY_PID_GAINS, DEFAULT_PID_GAINS, print_pid_for_keil


def run_unified_simulation(pid_gains=None, params=None):
    """运行统一的仿真
    
    Args:
        pid_gains: PID参数（默认使用MY_PID_GAINS）
        params: 机体参数（默认使用DEFAULT_UNIFIED_PARAMS）
    """
    
    # 1. 使用参数
    if pid_gains is None:
        pid_gains = MY_PID_GAINS  # 使用pid_params.py中定义的参数
    if params is None:
        params = DEFAULT_UNIFIED_PARAMS
    
    # 2. 初始化仿真组件
    dt = 0.002  # 2ms，与Keil工程的控制频率匹配
    dynamics = UnifiedQuadrotorDynamics(params)
    controller = UnifiedUavController(params, dt, pid_gains, use_physical_units=True)
    noise = SensorNoise()
    sensors = NoisyReadings(noise)
    
    # 3. 仿真参数
    t_end = 20.0
    steps = int(t_end / dt)
    
    # 4. 数据记录
    time_history = []
    pos_history = []
    angles_history = []
    pwm_history = []
    thrust_history = []
    
    # 5. 设置飞行任务
    hover_throttle = params.throttle_min + 400  # 典型悬停油门
    
    # 初始化
    t = 0.0
    dynamics.reset()
    controller.reset()
    
    print("开始统一仿真...")
    print(f"使用参数：")
    print(f"  质量: {params.mass} kg")
    print(f"  臂长: {params.arm_length} m")
    print(f"  悬停油门: {hover_throttle} μs")
    print(f"\nPID参数预览：")
    print(f"  角度环: Roll P={pid_gains.roll_angle_kp}, Pitch P={pid_gains.pitch_angle_kp}")
    print(f"  速率环: Roll P={pid_gains.roll_rate_kp}, Pitch P={pid_gains.pitch_rate_kp}")
    print("")
    
    # 6. 主仿真循环
    for step in range(steps):
        # 获取当前状态
        true_state = {
            "pos": dynamics.pos,
            "vel": dynamics.vel,
            "angles": dynamics.angles,
            "omega": dynamics.omega,
            "z": dynamics.pos[2],
            "vz": dynamics.vel[2],
        }
        
        # 添加传感器噪声
        meas = sensors.add_noise(true_state)
        
        # 生成控制指令
        if t < 2.0:
            # 前2秒：起飞准备
            throttle_cmd = params.motor_pwm_min
            angles_sp = np.zeros(3)
        elif t < 5.0:
            # 2-5秒：起飞到1米高度
            throttle_cmd = hover_throttle + 100  # 增加油门起飞
            angles_sp = np.zeros(3)
        elif t < 10.0:
            # 5-10秒：悬停
            throttle_cmd = hover_throttle
            angles_sp = np.zeros(3)
        elif t < 15.0:
            # 10-15秒：Roll轴倾斜测试
            throttle_cmd = hover_throttle
            angles_sp = np.array([np.deg2rad(10), 0, 0])  # 10度roll
        else:
            # 15秒后：回到悬停
            throttle_cmd = hover_throttle
            angles_sp = np.zeros(3)
        
        # 运行控制器（输出物理量）
        total_thrust, body_torque = controller.run(angles_sp, throttle_cmd, meas)
        
        # 分配到各电机（X型混控）
        from mixer_x_type import allocate_thrusts_x_type
        motor_thrusts = allocate_thrusts_x_type(total_thrust, body_torque, params)
        
        # 转换为PWM
        motor_pwm = np.zeros(4)
        for i in range(4):
            from unified_params import thrust_to_pwm
            motor_pwm[i] = thrust_to_pwm(motor_thrusts[i], params)
        
        # 执行动力学仿真
        state = dynamics.step(motor_pwm, dt)
        
        # 记录数据
        time_history.append(t)
        pos_history.append(state["pos"].copy())
        angles_history.append(state["angles"].copy())
        pwm_history.append(state["rotor_pwm"].copy())
        thrust_history.append(state["rotor_thrusts"].copy())
        
        # 打印进度
        if step % 1000 == 0:
            print(f"时间: {t:.1f}s, 高度: {dynamics.pos[2]:.2f}m, "
                  f"Roll: {np.rad2deg(dynamics.angles[0]):.1f}°, "
                  f"Pitch: {np.rad2deg(dynamics.angles[1]):.1f}°")
        
        t += dt
    
    # 7. 转换为数组便于绘图
    time_history = np.array(time_history)
    pos_history = np.array(pos_history)
    angles_history = np.array(angles_history)
    pwm_history = np.array(pwm_history)
    thrust_history = np.array(thrust_history)
    
    # 8. 绘制结果
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    fig.suptitle('四轴飞行器仿真结果（X型布局）', fontsize=16)
    
    # 位置
    ax = axes[0, 0]
    ax.plot(time_history, pos_history[:, 0], 'r-', label='X')
    ax.plot(time_history, pos_history[:, 1], 'g-', label='Y')
    ax.plot(time_history, pos_history[:, 2], 'b-', label='Z')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('位置 (m)')
    ax.set_title('位置轨迹')
    ax.legend()
    ax.grid(True)
    
    # 姿态角
    ax = axes[0, 1]
    ax.plot(time_history, np.rad2deg(angles_history[:, 0]), 'r-', label='Roll')
    ax.plot(time_history, np.rad2deg(angles_history[:, 1]), 'g-', label='Pitch')
    ax.plot(time_history, np.rad2deg(angles_history[:, 2]), 'b-', label='Yaw')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('角度 (°)')
    ax.set_title('姿态角')
    ax.legend()
    ax.grid(True)
    
    # PWM输出
    ax = axes[1, 0]
    for i in range(4):
        ax.plot(time_history, pwm_history[:, i], label=f'电机{i+1}')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('PWM (μs)')
    ax.set_title('电机PWM输出')
    ax.legend()
    ax.grid(True)
    ax.set_ylim([900, 2100])
    
    # 推力输出
    ax = axes[1, 1]
    for i in range(4):
        ax.plot(time_history, thrust_history[:, i], label=f'电机{i+1}')
    ax.plot(time_history, np.sum(thrust_history, axis=1), 'k--', label='总推力')
    ax.axhline(y=params.mass * params.gravity, color='r', linestyle=':', label='悬停推力')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('推力 (N)')
    ax.set_title('电机推力')
    ax.legend()
    ax.grid(True)
    
    # PWM差异（展示混控效果）
    ax = axes[2, 0]
    pwm_mean = np.mean(pwm_history, axis=1)
    for i in range(4):
        ax.plot(time_history, pwm_history[:, i] - pwm_mean, label=f'电机{i+1}')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('PWM偏差 (μs)')
    ax.set_title('电机PWM偏差（相对于平均值）')
    ax.legend()
    ax.grid(True)
    
    # 3D轨迹
    ax = axes[2, 1]
    ax.remove()
    ax = fig.add_subplot(3, 2, 6, projection='3d')
    ax.plot(pos_history[:, 0], pos_history[:, 1], pos_history[:, 2], 'b-')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D飞行轨迹')
    
    plt.tight_layout()
    plt.show()
    
    # 9. 输出PID参数总结
    print("\n仿真完成！")
    print("\n当前使用的PID参数（可直接复制到Keil工程）：")
    print_pid_for_keil(pid_gains)
    
    # 10. 性能评估
    print("\n性能指标：")
    # Roll通道阶跃响应分析（10-15秒）
    step_start_idx = int(10.0 / dt)
    step_end_idx = int(15.0 / dt)
    roll_response = angles_history[step_start_idx:step_end_idx, 0]
    
    # 计算上升时间（10%到90%）
    target_angle = np.deg2rad(10)
    rise_10 = 0.1 * target_angle
    rise_90 = 0.9 * target_angle
    rise_indices = np.where((roll_response >= rise_10) & (roll_response <= rise_90))[0]
    if len(rise_indices) > 0:
        rise_time = (rise_indices[-1] - rise_indices[0]) * dt
        print(f"  Roll上升时间: {rise_time:.3f}s")
    
    # 计算超调量
    max_angle = np.max(roll_response)
    overshoot = (max_angle - target_angle) / target_angle * 100
    print(f"  Roll超调量: {overshoot:.1f}%")
    
    # 稳态误差
    steady_state = roll_response[-int(1.0/dt):]  # 最后1秒
    steady_error = np.abs(np.mean(steady_state) - target_angle)
    print(f"  Roll稳态误差: {np.rad2deg(steady_error):.2f}°")
    
    return time_history, pos_history, angles_history, pwm_history


def compare_pid_settings():
    """比较不同PID设置的效果"""
    from pid_params import DEFAULT_PID_GAINS, CONSERVATIVE_PID_GAINS, AGGRESSIVE_PID_GAINS
    
    pid_settings = [
        ("默认参数", DEFAULT_PID_GAINS),
        ("保守参数", CONSERVATIVE_PID_GAINS),
        ("激进参数", AGGRESSIVE_PID_GAINS),
    ]
    
    print("比较不同PID参数设置...")
    print("每个仿真运行5秒\n")
    
    for name, gains in pid_settings:
        print(f"\n运行 {name}...")
        # 这里可以运行简短的仿真并记录关键指标
        # 为了演示，只打印参数
        print(f"  Roll P增益: 角度环={gains.roll_angle_kp}, 速率环={gains.roll_rate_kp}")


if __name__ == "__main__":
    # 运行主仿真
    run_unified_simulation()
    
    # 可选：比较不同参数
    # compare_pid_settings()