"""
统一的四轴飞行器参数定义
用于连接仿真程序与实际Keil工程

这个文件定义了实际四轴飞行器的物理参数和控制参数，
确保仿真结果可以直接应用到实际飞行器上。
"""

from dataclasses import dataclass
from typing import Tuple
import numpy as np


@dataclass
class UnifiedQuadParams:
    """统一的四轴飞行器参数"""
    
    # ===== 机体物理参数 =====
    # 这些参数需要通过实际测量获得
    mass: float = 1.2                  # 质量 (kg) - 需要用电子秤测量
    arm_length: float = 0.18           # 电机臂长 (m) - 从机体中心到电机轴的距离
    
    # 转动惯量 (kg*m^2) - 可以通过CAD模型或悬挂法测量
    inertia_xx: float = 0.015          # 绕x轴（roll）转动惯量
    inertia_yy: float = 0.015          # 绕y轴（pitch）转动惯量  
    inertia_zz: float = 0.028          # 绕z轴（yaw）转动惯量
    
    gravity: float = 9.81              # 重力加速度 (m/s^2)
    
    # ===== 电机参数 =====
    # PWM范围（与Keil工程PID_Config.h一致）
    motor_pwm_min: int = 1000          # 最小PWM值 (μs)
    motor_pwm_max: int = 2000          # 最大PWM值 (μs)
    
    # 电机推力参数（需要通过推力测试台标定）
    # 推力(N) = motor_thrust_coeff * (PWM - motor_pwm_min)^2 + motor_thrust_offset
    motor_thrust_coeff: float = 7.0e-6  # 推力系数
    motor_thrust_offset: float = 0.0     # 推力偏置
    motor_thrust_min: float = 0.0        # 最小推力 (N)
    motor_thrust_max: float = 7.0        # 单电机最大推力 (N)
    
    # 偏航力矩系数 (N*m/N) - 反映螺旋桨产生的反扭矩
    # 可通过固定机体只让单个电机运转测量
    yaw_moment_coeff: float = 1.5e-3
    
    # 电机响应时间常数 (s) - 一阶滞后模型
    motor_time_constant: float = 0.035
    
    # ===== 空气动力参数 =====
    # 简化的线性阻力系数 (N/(m/s))
    body_drag_xyz: Tuple[float, float, float] = (0.15, 0.15, 0.25)
    
    # ===== 控制参数（与Keil工程保持一致）=====
    throttle_min: int = 1100           # 油门激活阈值
    angle_max_deg: float = 30.0        # 最大倾斜角度 (度)
    yaw_rate_max_deg: float = 114.6    # 最大偏航角速度 (度/s) ≈ 2.0 rad/s


@dataclass 
class MotorMapping:
    """电机映射关系（X型布局）"""
    # Keil工程中的电机顺序
    MOTOR_FRONT_RIGHT = 1  # M1 - 前右
    MOTOR_BACK_RIGHT = 2   # M2 - 后右  
    MOTOR_BACK_LEFT = 3    # M3 - 后左
    MOTOR_FRONT_LEFT = 4   # M4 - 前左
    
    # 电机旋转方向（用于偏航控制）
    # +1: 顺时针（从上往下看）
    # -1: 逆时针（从上往下看）
    motor_directions = {
        1: -1,  # M1 顺时针
        2: +1,  # M2 逆时针
        3: -1,  # M3 顺时针
        4: +1,  # M4 逆时针
    }


def pwm_to_thrust(pwm: float, params: UnifiedQuadParams) -> float:
    """将PWM值转换为推力
    
    Args:
        pwm: PWM值 (μs)
        params: 统一参数
        
    Returns:
        推力 (N)
    """
    if pwm <= params.motor_pwm_min:
        return params.motor_thrust_min
    if pwm >= params.motor_pwm_max:
        return params.motor_thrust_max
        
    # 二次模型（常见的推力-PWM关系）
    pwm_normalized = pwm - params.motor_pwm_min
    thrust = params.motor_thrust_coeff * pwm_normalized**2 + params.motor_thrust_offset
    
    return np.clip(thrust, params.motor_thrust_min, params.motor_thrust_max)


def thrust_to_pwm(thrust: float, params: UnifiedQuadParams) -> int:
    """将推力转换为PWM值
    
    Args:
        thrust: 推力 (N)
        params: 统一参数
        
    Returns:
        PWM值 (μs)
    """
    if thrust <= params.motor_thrust_min:
        return params.motor_pwm_min
    if thrust >= params.motor_thrust_max:
        return params.motor_pwm_max
        
    # 反解二次方程
    pwm_normalized = np.sqrt((thrust - params.motor_thrust_offset) / params.motor_thrust_coeff)
    pwm = int(pwm_normalized + params.motor_pwm_min)
    
    return np.clip(pwm, params.motor_pwm_min, params.motor_pwm_max)


# 创建默认参数实例
DEFAULT_UNIFIED_PARAMS = UnifiedQuadParams()
