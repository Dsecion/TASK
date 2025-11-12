"""
四轴飞行器仿真包（统一版本）
使用X型布局，与Keil工程兼容
"""

from .unified_params import UnifiedQuadParams, DEFAULT_UNIFIED_PARAMS
from .controller_unified import UnifiedUavController, UnifiedAttitudeGains
from .dynamics_unified import UnifiedQuadrotorDynamics
from .mixer_x_type import allocate_thrusts_x_type, motor_mixing_to_pwm
from .pid import PID, PIDLimits
from .sensors import SensorNoise, NoisyReadings
from .pid_param_converter import generate_keil_config
from .pid_params import (
    DEFAULT_PID_GAINS,
    CONSERVATIVE_PID_GAINS, 
    AGGRESSIVE_PID_GAINS,
    MY_PID_GAINS,
    print_pid_for_keil
)

__all__ = [
    # 参数定义
    'UnifiedQuadParams',
    'DEFAULT_UNIFIED_PARAMS',
    # 控制器
    'UnifiedUavController',
    'UnifiedAttitudeGains',
    # 动力学
    'UnifiedQuadrotorDynamics',
    # 混控
    'allocate_thrusts_x_type',
    'motor_mixing_to_pwm',
    # PID基础
    'PID',
    'PIDLimits',
    # 传感器
    'SensorNoise',
    'NoisyReadings',
    # PID参数
    'DEFAULT_PID_GAINS',
    'CONSERVATIVE_PID_GAINS',
    'AGGRESSIVE_PID_GAINS',
    'MY_PID_GAINS',
    'print_pid_for_keil',
    # 工具
    'generate_keil_config',
]

__version__ = '2.1.0'  # 独立PID参数版本