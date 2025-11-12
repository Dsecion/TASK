"""
X型四旋翼混控器
与Keil工程保持一致的电机布局和混控算法
"""

from __future__ import annotations

import numpy as np
from .dynamics import QuadParams
from .unified_params import UnifiedQuadParams, pwm_to_thrust, thrust_to_pwm


def allocate_thrusts_x_type(
    total_thrust: float,
    body_torque: np.ndarray,
    params: QuadParams | UnifiedQuadParams,
) -> np.ndarray:
    """
    X型四旋翼混控算法（与Keil工程一致）
    
    电机布局（X型配置）：
        M4(前左)  M1(前右)
             ╲    ╱ 
              ╳   
             ╱    ╲  
        M3(后左)  M2(后右)
    
    电机旋转方向：
    M1, M3: 顺时针（从上往下看）
    M2, M4: 逆时针（从上往下看）
    
    Args:
        total_thrust: 总推力 (N)
        body_torque: 机体力矩 [τx, τy, τz] (N*m)
        params: 四旋翼参数
        
    Returns:
        各电机推力 [f1, f2, f3, f4] (N)
    """
    tau = np.asarray(body_torque, dtype=float).reshape(3)
    arm = params.arm_length
    k = params.yaw_moment_coeff
    
    # 构建X型四旋翼的分配矩阵
    # 与Keil工程PWM_Motor_Mixing函数对应：
    # motor1 = throttle - roll + pitch - yaw  (前右)
    # motor2 = throttle - roll - pitch + yaw  (后右)
    # motor3 = throttle + roll - pitch - yaw  (后左)
    # motor4 = throttle + roll + pitch + yaw  (前左)
    
    # 推力和力矩的关系矩阵
    # [T, τx, τy, τz]^T = B * [f1, f2, f3, f4]^T
    B = np.array([
        [1.0,      1.0,      1.0,      1.0     ],  # 总推力
        [-arm/np.sqrt(2), -arm/np.sqrt(2), arm/np.sqrt(2), arm/np.sqrt(2)],  # Roll力矩 (τx)
        [arm/np.sqrt(2), -arm/np.sqrt(2), -arm/np.sqrt(2), arm/np.sqrt(2)],  # Pitch力矩 (τy)
        [-k,       k,       -k,        k       ],  # Yaw力矩 (τz)
    ])
    
    # 期望的推力和力矩向量
    w = np.array([total_thrust, tau[0], tau[1], tau[2]], dtype=float)
    
    # 求解线性方程组 B * f = w
    f = np.linalg.solve(B, w)
    
    # 饱和处理
    if hasattr(params, 'motor_thrust_min'):
        f = np.clip(f, params.motor_thrust_min, params.motor_thrust_max)
    else:
        # 兼容原有参数
        f = np.clip(f, 0.0, params.motor_thrust_max)
    
    return f


def motor_mixing_to_pwm(
    throttle_pwm: float,
    roll_output: float, 
    pitch_output: float,
    yaw_output: float,
    params: UnifiedQuadParams,
) -> np.ndarray:
    """
    将Keil工程风格的混控输入转换为PWM输出
    完全复现Keil工程的PWM_Motor_Mixing函数
    
    Args:
        throttle_pwm: 基础油门PWM值 (μs)
        roll_output: Roll轴PID输出（PWM增量）
        pitch_output: Pitch轴PID输出（PWM增量）
        yaw_output: Yaw轴PID输出（PWM增量）
        params: 统一参数
        
    Returns:
        各电机PWM值 [motor1, motor2, motor3, motor4] (μs)
    """
    # 直接应用Keil工程的混控算法
    motor1 = throttle_pwm - roll_output + pitch_output - yaw_output  # 前右
    motor2 = throttle_pwm - roll_output - pitch_output + yaw_output  # 后右
    motor3 = throttle_pwm + roll_output - pitch_output - yaw_output  # 后左
    motor4 = throttle_pwm + roll_output + pitch_output + yaw_output  # 前左
    
    # PWM饱和处理
    motor1 = np.clip(motor1, params.motor_pwm_min, params.motor_pwm_max)
    motor2 = np.clip(motor2, params.motor_pwm_min, params.motor_pwm_max)
    motor3 = np.clip(motor3, params.motor_pwm_min, params.motor_pwm_max)
    motor4 = np.clip(motor4, params.motor_pwm_min, params.motor_pwm_max)
    
    return np.array([motor1, motor2, motor3, motor4], dtype=float)


def convert_torque_to_pwm_increment(
    body_torque: np.ndarray,
    params: UnifiedQuadParams,
    current_throttle_pwm: float = 1500,
) -> np.ndarray:
    """
    将物理力矩转换为PWM增量（用于仿真到实际的参数映射）
    
    Args:
        body_torque: 机体力矩 [τx, τy, τz] (N*m)
        params: 统一参数
        current_throttle_pwm: 当前油门PWM基准值
        
    Returns:
        PWM增量 [roll_increment, pitch_increment, yaw_increment]
    """
    # 将力矩转换为等效的推力变化
    # 这需要考虑臂长和混控矩阵的影响
    arm = params.arm_length
    k = params.yaw_moment_coeff
    
    # 对于X型布局，力矩到推力变化的关系
    # Roll力矩：Δf = τx / (2 * arm/√2) 
    # Pitch力矩：Δf = τy / (2 * arm/√2)
    # Yaw力矩：Δf = τz / (4 * k)
    
    thrust_increment_roll = abs(body_torque[0]) / (2 * arm / np.sqrt(2))
    thrust_increment_pitch = abs(body_torque[1]) / (2 * arm / np.sqrt(2))
    thrust_increment_yaw = abs(body_torque[2]) / (4 * k)
    
    # 将推力增量转换为PWM增量（线性近似）
    # 在悬停点附近的线性化
    hover_thrust = params.mass * params.gravity / 4  # 单电机悬停推力
    hover_pwm = thrust_to_pwm(hover_thrust, params)
    
    # 计算推力-PWM斜率（在悬停点）
    delta_thrust = 0.1  # N
    slope = (thrust_to_pwm(hover_thrust + delta_thrust, params) - 
             thrust_to_pwm(hover_thrust - delta_thrust, params)) / (2 * delta_thrust)
    
    # 转换为PWM增量
    pwm_increment_roll = thrust_increment_roll * slope * np.sign(body_torque[0])
    pwm_increment_pitch = thrust_increment_pitch * slope * np.sign(body_torque[1])
    pwm_increment_yaw = thrust_increment_yaw * slope * np.sign(body_torque[2])
    
    return np.array([pwm_increment_roll, pwm_increment_pitch, pwm_increment_yaw])
