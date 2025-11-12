"""
统一的四旋翼动力学模型
使用X型布局，与Keil工程保持一致
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple
import numpy as np

from .unified_params import UnifiedQuadParams, pwm_to_thrust


class UnifiedQuadrotorDynamics:
    """
    使用X型布局的四旋翼动力学模型
    与Keil工程的电机编号和混控逻辑保持一致
    
    电机布局：
        M4(前左)  M1(前右)
             ╲    ╱ 
              ╳   
             ╱    ╲  
        M3(后左)  M2(后右)
    """
    
    def __init__(self, params: UnifiedQuadParams) -> None:
        self.params = params
        self.mass = params.mass
        self.J = np.diag([params.inertia_xx, params.inertia_yy, params.inertia_zz])
        self.J_inv = np.linalg.inv(self.J)
        self.reset()
    
    def reset(self) -> None:
        """重置状态"""
        self.pos = np.zeros(3)          # 位置 [x, y, z]
        self.vel = np.zeros(3)          # 速度 [vx, vy, vz]
        self.angles = np.zeros(3)       # 欧拉角 [roll, pitch, yaw]
        self.omega = np.zeros(3)        # 角速度 [p, q, r]
        self.rotor_thrusts = np.zeros(4)  # 各电机推力 (N)
        self.rotor_pwm = np.ones(4) * self.params.motor_pwm_min  # PWM值
    
    @staticmethod
    def _rotation_matrix(angles: np.ndarray) -> np.ndarray:
        """计算从机体坐标系到世界坐标系的旋转矩阵"""
        phi, theta, psi = angles
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)
        cpsi, spsi = np.cos(psi), np.sin(psi)
        
        # ZYX欧拉角旋转顺序
        Rz = np.array([[cpsi, -spsi, 0.0],
                       [spsi,  cpsi, 0.0],
                       [0.0,   0.0,  1.0]])
        Ry = np.array([[ cth, 0.0, sth],
                       [ 0.0, 1.0, 0.0],
                       [-sth, 0.0, cth]])
        Rx = np.array([[1.0, 0.0,  0.0],
                       [0.0, cphi, -sphi],
                       [0.0, sphi,  cphi]])
        return Rz @ Ry @ Rx
    
    @staticmethod
    def _euler_rate_matrix(angles: np.ndarray) -> np.ndarray:
        """欧拉角速率到机体角速度的转换矩阵"""
        phi, theta, _ = angles
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)
        
        # 避免奇点
        if abs(cth) < 1e-6:
            cth = 1e-6
        
        E = np.array([
            [1.0, sphi * sth / cth, cphi * sth / cth],
            [0.0, cphi,            -sphi],
            [0.0, sphi / cth,       cphi / cth],
        ])
        return E
    
    def _motor_dynamics(self, pwm_cmd: np.ndarray, dt: float) -> None:
        """电机动力学（一阶滞后）"""
        tau = max(1e-4, self.params.motor_time_constant)
        
        # PWM响应
        dpwm = (pwm_cmd - self.rotor_pwm) / tau
        self.rotor_pwm += dpwm * dt
        self.rotor_pwm = np.clip(
            self.rotor_pwm,
            self.params.motor_pwm_min,
            self.params.motor_pwm_max
        )
        
        # 转换为推力
        for i in range(4):
            self.rotor_thrusts[i] = pwm_to_thrust(self.rotor_pwm[i], self.params)
    
    def _forces_and_torques_x_type(self) -> Tuple[np.ndarray, np.ndarray]:
        """计算X型布局的力和力矩"""
        # 各电机推力
        f1, f2, f3, f4 = self.rotor_thrusts
        
        # 总推力（机体z轴方向）
        T = float(np.sum(self.rotor_thrusts))
        
        # X型布局的力矩计算
        arm = self.params.arm_length
        k = self.params.yaw_moment_coeff
        
        # 力矩计算（与Keil工程PWM_Motor_Mixing对应）
        # tau_x (roll): M3,M4产生正力矩，M1,M2产生负力矩
        # tau_y (pitch): M1,M4产生正力矩，M2,M3产生负力矩
        # tau_z (yaw): M2,M4逆时针(正)，M1,M3顺时针(负)
        
        # 考虑X型的45度角
        arm_eff = arm / np.sqrt(2)
        
        tau_x = arm_eff * (-f1 - f2 + f3 + f4)
        tau_y = arm_eff * (f1 - f2 - f3 + f4)
        tau_z = k * (-f1 + f2 - f3 + f4)
        
        tau = np.array([tau_x, tau_y, tau_z])
        
        # 世界坐标系下的力
        R = self._rotation_matrix(self.angles)
        thrust_world = R @ np.array([0.0, 0.0, T])
        gravity = np.array([0.0, 0.0, -self.params.gravity * self.mass])
        
        # 空气阻力
        drag_body = -np.array(self.params.body_drag_xyz) * (R.T @ self.vel)
        drag_world = R @ drag_body
        
        force_world = thrust_world + gravity + drag_world
        return force_world, tau
    
    def _derivatives(self, state: np.ndarray, f_world: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """计算状态导数"""
        pos = state[0:3]
        vel = state[3:6]
        angles = state[6:9]
        omega = state[9:12]
        
        # 位置导数
        pos_dot = vel
        
        # 速度导数
        vel_dot = f_world / self.mass
        
        # 姿态导数
        E = self._euler_rate_matrix(angles)
        angles_dot = E @ omega
        
        # 角速度导数
        omega_dot = self.J_inv @ (tau - np.cross(omega, self.J @ omega))
        
        return np.concatenate([pos_dot, vel_dot, angles_dot, omega_dot])
    
    def step(self, motor_pwm_cmd: np.ndarray, dt: float) -> Dict[str, np.ndarray]:
        """
        执行一步仿真
        
        Args:
            motor_pwm_cmd: 电机PWM指令 [m1, m2, m3, m4] (μs)
            dt: 时间步长 (s)
            
        Returns:
            状态字典
        """
        motor_pwm_cmd = np.asarray(motor_pwm_cmd, dtype=float).reshape(4)
        
        # 更新电机状态
        self._motor_dynamics(motor_pwm_cmd, dt)
        
        # 准备状态向量
        state = np.concatenate([self.pos, self.vel, self.angles, self.omega])
        
        def derivatives(s: np.ndarray) -> np.ndarray:
            # 临时更新状态以计算力和力矩
            angles_saved = self.angles.copy()
            vel_saved = self.vel.copy()
            self.angles = s[6:9]
            self.vel = s[3:6]
            f_world, tau = self._forces_and_torques_x_type()
            self.angles = angles_saved
            self.vel = vel_saved
            return self._derivatives(s, f_world, tau)
        
        # RK4积分
        k1 = derivatives(state)
        k2 = derivatives(state + 0.5 * dt * k1)
        k3 = derivatives(state + 0.5 * dt * k2)
        k4 = derivatives(state + dt * k3)
        new_state = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        
        # 更新状态
        self.pos = new_state[0:3]
        self.vel = new_state[3:6]
        self.angles = new_state[6:9]
        self.omega = new_state[9:12]
        
        return {
            "pos": self.pos.copy(),
            "vel": self.vel.copy(),
            "angles": self.angles.copy(),
            "omega": self.omega.copy(),
            "rotor_thrusts": self.rotor_thrusts.copy(),
            "rotor_pwm": self.rotor_pwm.copy(),
        }
    
    def step_with_thrust_input(self, rotor_thrust_cmd: np.ndarray, dt: float) -> Dict[str, np.ndarray]:
        """使用推力输入（用于兼容原仿真接口）"""
        # 将推力转换为PWM
        pwm_cmd = np.zeros(4)
        for i in range(4):
            from .unified_params import thrust_to_pwm
            pwm_cmd[i] = thrust_to_pwm(rotor_thrust_cmd[i], self.params)
        
        return self.step(pwm_cmd, dt)
