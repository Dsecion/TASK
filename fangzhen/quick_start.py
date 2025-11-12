"""
快速开始示例
演示如何使用统一的四轴飞行器仿真系统
"""

import numpy as np
from unified_params import UnifiedQuadParams
from controller_unified import UnifiedUavController
from dynamics_unified import UnifiedQuadrotorDynamics
from sensors import SensorNoise, NoisyReadings
from plots import plot_results
from pid_params import MY_PID_GAINS  # 使用pid_params.py中的参数


def main():
    print("=== 四轴飞行器仿真系统（统一版）===")
    print("使用X型布局，与Keil工程兼容\n")
    
    # 1. 配置参数（这些应该与您的实际飞行器匹配）
    params = UnifiedQuadParams(
        mass=1.2,                    # 质量 kg
        arm_length=0.18,             # 臂长 m
        motor_thrust_max=7.0,        # 单电机最大推力 N
    )
    
    # 2. 使用pid_params.py中定义的PID参数
    pid_gains = MY_PID_GAINS  # 您可以在pid_params.py中调整这些参数
    
    # 3. 初始化组件
    dt = 0.002  # 2ms控制周期
    dynamics = UnifiedQuadrotorDynamics(params)
    controller = UnifiedUavController(params, dt, pid_gains)
    noise = SensorNoise(seed=42)
    sensors = NoisyReadings(noise)
    
    # 4. 设置仿真时间
    t_sim = 10.0  # 10秒仿真
    steps = int(t_sim / dt)
    
    # 5. 初始化日志
    logs = {
        "t": np.zeros(steps),
        "pos": np.zeros((steps, 3)),
        "angles": np.zeros((steps, 3)),
        "angles_sp": np.zeros((steps, 3)),
        "z_sp": np.zeros(steps),
        "thrust_cmd": np.zeros(steps),
        "tau_cmd": np.zeros((steps, 3)),
        "motor_pwm": np.zeros((steps, 4)),
    }
    
    # 6. 运行仿真
    print("开始仿真...")
    print(f"PID设置: Roll P={pid_gains.roll_angle_kp}/{pid_gains.roll_rate_kp}")
    hover_throttle = 1500  # 典型悬停油门 μs
    
    for i in range(steps):
        t = i * dt
        
        # 获取测量值
        true_state = {
            "pos": dynamics.pos,
            "vel": dynamics.vel,
            "angles": dynamics.angles,
            "omega": dynamics.omega,
            "z": dynamics.pos[2],
            "vz": dynamics.vel[2],
        }
        meas = sensors.add_noise(true_state)
        
        # 生成控制指令
        if t < 1.0:
            throttle = 1000  # 初始低油门
            angles_sp = np.zeros(3)
        elif t < 3.0:
            throttle = hover_throttle + 100  # 起飞
            angles_sp = np.zeros(3)
        elif t < 6.0:
            throttle = hover_throttle  # 悬停
            angles_sp = np.zeros(3)
        elif t < 8.0:
            throttle = hover_throttle
            angles_sp = np.array([np.deg2rad(10), 0, 0])  # Roll 10度
        else:
            throttle = hover_throttle
            angles_sp = np.zeros(3)
        
        # 运行控制器
        total_thrust, body_torque = controller.run(angles_sp, float(throttle), meas)
        
        # 分配到电机
        from mixer_x_type import allocate_thrusts_x_type
        from unified_params import thrust_to_pwm
        motor_thrusts = allocate_thrusts_x_type(total_thrust, body_torque, params)
        motor_pwm = np.array([thrust_to_pwm(f, params) for f in motor_thrusts])
        
        # 执行动力学
        state = dynamics.step(motor_pwm, dt)
        
        # 记录数据
        logs["t"][i] = t
        logs["pos"][i] = state["pos"]
        logs["angles"][i] = state["angles"]
        logs["angles_sp"][i] = angles_sp
        logs["z_sp"][i] = 1.0 if t > 1.0 else 0.0  # 目标高度
        logs["thrust_cmd"][i] = total_thrust
        logs["tau_cmd"][i] = body_torque
        logs["motor_pwm"][i] = motor_pwm
        
        # 进度显示
        if i % 500 == 0:
            print(f"  t={t:.1f}s, 高度={dynamics.pos[2]:.2f}m, "
                  f"Roll={np.rad2deg(dynamics.angles[0]):.1f}°")
    
    print("\n仿真完成！")
    
    # 7. 显示可复制到Keil的PID参数
    from pid_params import print_pid_for_keil
    print("\n可用于Keil工程的PID参数：")
    print_pid_for_keil(pid_gains)
    
    # 8. 绘制结果
    plot_results(logs)
    
    return logs


if __name__ == "__main__":
    main()