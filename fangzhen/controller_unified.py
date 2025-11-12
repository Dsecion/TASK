"""
统一的控制器实现
使仿真PID参数与Keil工程直接对应
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple
import numpy as np

from .pid import PID, PIDLimits
from .unified_params import UnifiedQuadParams, pwm_to_thrust, thrust_to_pwm, convert_torque_to_pwm_increment


@dataclass
class UnifiedAttitudeGains:
    """与Keil工程PID_Config.h对应的姿态控制增益"""
    # 外环角度控制（输入：弧度，输出：弧度/秒）
    roll_angle_kp: float = 2.0      # ANGLE_ROLL_KP
    roll_angle_ki: float = 0.0      # ANGLE_ROLL_KI
    roll_angle_kd: float = 0.5      # ANGLE_ROLL_KD
    
    pitch_angle_kp: float = 2.0     # ANGLE_PITCH_KP
    pitch_angle_ki: float = 0.0     # ANGLE_PITCH_KI
    pitch_angle_kd: float = 0.5     # ANGLE_PITCH_KD
    
    yaw_angle_kp: float = 1.5       # ANGLE_YAW_KP
    yaw_angle_ki: float = 0.0       # ANGLE_YAW_KI
    yaw_angle_kd: float = 0.3       # ANGLE_YAW_KD
    
    # 内环角速度控制（输入：弧度/秒，输出：PWM增量）
    roll_rate_kp: float = 50.0      # RATE_ROLL_KP
    roll_rate_ki: float = 0.5       # RATE_ROLL_KI
    roll_rate_kd: float = 5.0       # RATE_ROLL_KD
    
    pitch_rate_kp: float = 50.0     # RATE_PITCH_KP
    pitch_rate_ki: float = 0.5      # RATE_PITCH_KI
    pitch_rate_kd: float = 5.0      # RATE_PITCH_KD
    
    yaw_rate_kp: float = 80.0       # RATE_YAW_KP
    yaw_rate_ki: float = 0.5        # RATE_YAW_KI
    yaw_rate_kd: float = 8.0        # RATE_YAW_KD
    
    # 输出限制（从PID_Config.h读取）
    angle_roll_output_max: float = 5.0      # rad/s
    angle_pitch_output_max: float = 5.0     # rad/s
    angle_yaw_output_max: float = 3.0       # rad/s
    
    rate_roll_output_max: float = 400.0     # PWM增量
    rate_pitch_output_max: float = 400.0    # PWM增量
    rate_yaw_output_max: float = 300.0      # PWM增量


@dataclass
class SimulationGains:
    """仿真使用的增益（物理单位）"""
    # 角度环输出角速度（rad/s）
    roll_angle_kp: float
    roll_angle_ki: float
    roll_angle_kd: float
    
    pitch_angle_kp: float
    pitch_angle_ki: float
    pitch_angle_kd: float
    
    yaw_angle_kp: float
    yaw_angle_ki: float
    yaw_angle_kd: float
    
    # 角速度环输出力矩（N*m）
    roll_rate_kp: float
    roll_rate_ki: float
    roll_rate_kd: float
    
    pitch_rate_kp: float
    pitch_rate_ki: float
    pitch_rate_kd: float
    
    yaw_rate_kp: float
    yaw_rate_ki: float
    yaw_rate_kd: float


def convert_to_simulation_gains(
    unified_gains: UnifiedAttitudeGains,
    params: UnifiedQuadParams,
) -> SimulationGains:
    """将Keil工程的PID参数转换为仿真使用的物理单位参数
    
    转换原理：
    - 角度环：直接使用，因为输入输出单位相同（rad -> rad/s）
    - 角速度环：需要将PWM增量转换为力矩
    """
    
    # 计算悬停状态的参考值
    hover_thrust = params.mass * params.gravity / 4  # 单电机悬停推力
    hover_pwm = thrust_to_pwm(hover_thrust, params)
    
    # 计算PWM到推力的局部线性化斜率
    delta_pwm = 50  # PWM增量用于计算斜率
    thrust_upper = pwm_to_thrust(hover_pwm + delta_pwm, params)
    thrust_lower = pwm_to_thrust(hover_pwm - delta_pwm, params)
    pwm_to_thrust_slope = (thrust_upper - thrust_lower) / (2 * delta_pwm)
    
    # 计算从PWM增量到力矩的转换系数
    # 对于X型布局：
    # Roll/Pitch力矩 = 2 * (arm/√2) * Δ推力
    # Yaw力矩 = 4 * k * Δ推力
    arm = params.arm_length
    k = params.yaw_moment_coeff
    
    pwm_to_roll_torque = 2 * (arm / np.sqrt(2)) * pwm_to_thrust_slope
    pwm_to_pitch_torque = 2 * (arm / np.sqrt(2)) * pwm_to_thrust_slope
    pwm_to_yaw_torque = 4 * k * pwm_to_thrust_slope
    
    return SimulationGains(
        # 角度环参数直接使用
        roll_angle_kp=unified_gains.roll_angle_kp,
        roll_angle_ki=unified_gains.roll_angle_ki,
        roll_angle_kd=unified_gains.roll_angle_kd,
        
        pitch_angle_kp=unified_gains.pitch_angle_kp,
        pitch_angle_ki=unified_gains.pitch_angle_ki,
        pitch_angle_kd=unified_gains.pitch_angle_kd,
        
        yaw_angle_kp=unified_gains.yaw_angle_kp,
        yaw_angle_ki=unified_gains.yaw_angle_ki,
        yaw_angle_kd=unified_gains.yaw_angle_kd,
        
        # 角速度环参数需要转换单位
        roll_rate_kp=unified_gains.roll_rate_kp * pwm_to_roll_torque,
        roll_rate_ki=unified_gains.roll_rate_ki * pwm_to_roll_torque,
        roll_rate_kd=unified_gains.roll_rate_kd * pwm_to_roll_torque,
        
        pitch_rate_kp=unified_gains.pitch_rate_kp * pwm_to_pitch_torque,
        pitch_rate_ki=unified_gains.pitch_rate_ki * pwm_to_pitch_torque,
        pitch_rate_kd=unified_gains.pitch_rate_kd * pwm_to_pitch_torque,
        
        yaw_rate_kp=unified_gains.yaw_rate_kp * pwm_to_yaw_torque,
        yaw_rate_ki=unified_gains.yaw_rate_ki * pwm_to_yaw_torque,
        yaw_rate_kd=unified_gains.yaw_rate_kd * pwm_to_yaw_torque,
    )


class UnifiedUavController:
    """统一的无人机控制器（与Keil工程兼容）"""
    
    def __init__(
        self,
        params: UnifiedQuadParams,
        dt: float,
        unified_gains: UnifiedAttitudeGains | None = None,
        use_physical_units: bool = True,
    ) -> None:
        self.params = params
        self.dt = float(dt)
        self.use_physical_units = use_physical_units
        
        gains = unified_gains or UnifiedAttitudeGains()
        
        if use_physical_units:
            # 仿真模式：转换为物理单位
            sim_gains = convert_to_simulation_gains(gains, params)
            self._init_simulation_pids(sim_gains, gains)
        else:
            # 硬件模式：直接使用PWM单位
            self._init_hardware_pids(gains)
        
        self._hover_thrust = params.mass * params.gravity
        self._hover_pwm = params.throttle_min + 400  # 典型悬停油门
    
    def _init_simulation_pids(self, sim_gains: SimulationGains, unified_gains: UnifiedAttitudeGains):
        """初始化仿真用的PID控制器（物理单位）"""
        # 角度环：输出角速度（rad/s）
        rate_limit = np.deg2rad(300.0)
        self.roll_angle_pid = PID(
            sim_gains.roll_angle_kp, sim_gains.roll_angle_ki, sim_gains.roll_angle_kd, self.dt,
            limits=PIDLimits(output_min=-unified_gains.angle_roll_output_max, 
                           output_max=unified_gains.angle_roll_output_max),
            deriv_filter_hz=10.0
        )
        self.pitch_angle_pid = PID(
            sim_gains.pitch_angle_kp, sim_gains.pitch_angle_ki, sim_gains.pitch_angle_kd, self.dt,
            limits=PIDLimits(output_min=-unified_gains.angle_pitch_output_max,
                           output_max=unified_gains.angle_pitch_output_max),
            deriv_filter_hz=10.0
        )
        self.yaw_angle_pid = PID(
            sim_gains.yaw_angle_kp, sim_gains.yaw_angle_ki, sim_gains.yaw_angle_kd, self.dt,
            limits=PIDLimits(output_min=-unified_gains.angle_yaw_output_max,
                           output_max=unified_gains.angle_yaw_output_max),
            deriv_filter_hz=10.0
        )
        
        # 角速度环：输出力矩（N*m）
        # 计算力矩限制
        max_thrust_diff = pwm_to_thrust(self.params.motor_pwm_max, self.params) - \
                          pwm_to_thrust(self.params.motor_pwm_min, self.params)
        torque_limit_roll = max_thrust_diff * 2 * (self.params.arm_length / np.sqrt(2))
        torque_limit_pitch = torque_limit_roll
        torque_limit_yaw = max_thrust_diff * 4 * self.params.yaw_moment_coeff
        
        self.p_rate_pid = PID(
            sim_gains.roll_rate_kp, sim_gains.roll_rate_ki, sim_gains.roll_rate_kd, self.dt,
            limits=PIDLimits(output_min=-torque_limit_roll, output_max=torque_limit_roll),
            deriv_filter_hz=30.0
        )
        self.q_rate_pid = PID(
            sim_gains.pitch_rate_kp, sim_gains.pitch_rate_ki, sim_gains.pitch_rate_kd, self.dt,
            limits=PIDLimits(output_min=-torque_limit_pitch, output_max=torque_limit_pitch),
            deriv_filter_hz=30.0
        )
        self.r_rate_pid = PID(
            sim_gains.yaw_rate_kp, sim_gains.yaw_rate_ki, sim_gains.yaw_rate_kd, self.dt,
            limits=PIDLimits(output_min=-torque_limit_yaw, output_max=torque_limit_yaw),
            deriv_filter_hz=30.0
        )
    
    def _init_hardware_pids(self, gains: UnifiedAttitudeGains):
        """初始化硬件用的PID控制器（PWM单位）"""
        # 这里的实现与Keil工程完全一致
        pass  # TODO: 如果需要在Python中直接输出PWM，可以实现此部分
    
    def reset(self) -> None:
        """重置所有PID控制器"""
        self.roll_angle_pid.reset()
        self.pitch_angle_pid.reset()
        self.yaw_angle_pid.reset()
        self.p_rate_pid.reset()
        self.q_rate_pid.reset()
        self.r_rate_pid.reset()
    
    def attitude_control(
        self,
        angles_sp: np.ndarray,
        angles_meas: np.ndarray,
        rates_meas: np.ndarray,
    ) -> Tuple[float, float, float]:
        """姿态控制（级联控制器）"""
        phi_sp, theta_sp, psi_sp = angles_sp
        phi, theta, psi = angles_meas
        p_meas, q_meas, r_meas = rates_meas
        
        # 角度环：计算期望角速度
        p_sp = self.roll_angle_pid.update(setpoint=float(phi_sp), measurement=float(phi))
        q_sp = self.pitch_angle_pid.update(setpoint=float(theta_sp), measurement=float(theta))
        
        # Yaw角处理（考虑周期性）
        yaw_err = self._wrap_yaw_error(float(psi_sp), float(psi))
        r_sp = self.yaw_angle_pid.update(setpoint=0.0, measurement=-yaw_err)
        
        # 角速度环：计算力矩或PWM增量
        if self.use_physical_units:
            # 输出力矩（N*m）
            tau_x = self.p_rate_pid.update(setpoint=float(p_sp), measurement=float(p_meas))
            tau_y = self.q_rate_pid.update(setpoint=float(q_sp), measurement=float(q_meas))
            tau_z = self.r_rate_pid.update(setpoint=float(r_sp), measurement=float(r_meas))
        else:
            # 输出PWM增量
            tau_x = self.p_rate_pid.update(setpoint=float(p_sp), measurement=float(p_meas))
            tau_y = self.q_rate_pid.update(setpoint=float(q_sp), measurement=float(q_meas))
            tau_z = self.r_rate_pid.update(setpoint=float(r_sp), measurement=float(r_meas))
            
        return float(tau_x), float(tau_y), float(tau_z)
    
    @staticmethod
    def _wrap_yaw_error(yaw_sp: float, yaw_meas: float) -> float:
        """计算yaw角误差（处理周期性）"""
        e = yaw_sp - yaw_meas
        e = (e + np.pi) % (2 * np.pi) - np.pi
        return e
    
    def run(
        self,
        angles_sp: np.ndarray,
        throttle_command: float,  # PWM值或推力值
        meas: Dict[str, np.ndarray | float],
    ) -> Tuple[float, np.ndarray]:
        """运行控制器
        
        Args:
            angles_sp: 期望姿态角 [roll, pitch, yaw] (rad)
            throttle_command: 油门指令（PWM μs 或 推力 N）
            meas: 测量值字典
            
        Returns:
            (total_thrust, body_torque) 或 (throttle_pwm, pwm_increments)
        """
        angles_meas = np.asarray(meas["angles"], dtype=float).reshape(3)
        omega_meas = np.asarray(meas["omega"], dtype=float).reshape(3)
        
        tau_x, tau_y, tau_z = self.attitude_control(angles_sp, angles_meas, omega_meas)
        
        if self.use_physical_units:
            # 返回物理量：总推力和力矩
            if throttle_command > 100:  # 假设是PWM值
                total_thrust = pwm_to_thrust(throttle_command, self.params) * 4
            else:
                total_thrust = throttle_command
            tau = np.array([tau_x, tau_y, tau_z], dtype=float)
            return total_thrust, tau
        else:
            # 返回PWM值
            return throttle_command, np.array([tau_x, tau_y, tau_z], dtype=float)
