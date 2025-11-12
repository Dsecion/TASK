# 统一的四轴飞行器仿真系统 - 详细指南

本系统解决了Python仿真与Keil工程之间的兼容性问题，使得在仿真中调试的PID参数可以直接应用到实际飞行器上。

## 系统架构

### 1. 独立的PID参数管理
- PID参数存储在 `pid_params.py` 文件中
- 提供多种预设：默认、保守、激进
- 支持自定义参数 `MY_PID_GAINS`

### 2. 统一的电机布局
- 采用X型四旋翼布局（与Keil工程一致）
- 电机编号：M1(前右)、M2(后右)、M3(后左)、M4(前左)
- 旋转方向：M1,M3顺时针，M2,M4逆时针

### 3. 自动单位转换
- 仿真内部使用物理单位（N, N·m）
- 自动转换为PWM输出（μs）
- PID参数保持与Keil一致的定义

## 使用流程

### Step 1: 配置机体参数

如果您的飞行器参数与默认不同，修改 `unified_params.py`：

```python
MY_QUAD_PARAMS = UnifiedQuadParams(
    mass=1.35,              # 实测质量 (kg)
    arm_length=0.225,       # 实测臂长 (m)
    motor_thrust_coeff=8.5e-6,  # 推力系数（需标定）
    # ... 其他参数
)
```

参考 `calibration_guide.md` 了解如何测量这些参数。

### Step 2: 调整PID参数

编辑 `pid_params.py` 中的 `MY_PID_GAINS`：

```python
MY_PID_GAINS = UnifiedAttitudeGains(
    # ===== 外环：角度控制 =====
    roll_angle_kp=2.5,      # 响应速度
    roll_angle_ki=0.05,     # 消除稳态误差
    roll_angle_kd=0.6,      # 减少超调
    
    # ===== 内环：角速度控制 =====  
    roll_rate_kp=60.0,      # 内环响应
    roll_rate_ki=0.8,       # 内环积分
    roll_rate_kd=6.0,       # 内环微分
    
    # ... 其他轴参数
)
```

### Step 3: 运行仿真测试

```python
# 使用自定义参数运行
python run_unified_simulation.py

# 或在代码中指定参数
from pid_params import MY_PID_GAINS, AGGRESSIVE_PID_GAINS
run_unified_simulation(pid_gains=AGGRESSIVE_PID_GAINS)
```

### Step 4: 分析结果

仿真会显示：
- 3D轨迹和姿态响应
- PWM输出和电机推力
- 性能指标（上升时间、超调量、稳态误差）

### Step 5: 导出到Keil

仿真结束后，复制显示的参数到 `PID_Config.h`：

```c
// ===== 外环PID参数 =====
#define ANGLE_ROLL_KP           2.5f
#define ANGLE_ROLL_KI           0.1f
#define ANGLE_ROLL_KD           0.6f
// ... 其他参数
```

或使用工具生成完整文件：

```python
from pid_params import MY_PID_GAINS
from pid_param_converter import generate_keil_config

generate_keil_config(MY_PID_GAINS, "PID_Config_new.h")
```

## PID调试技巧

### 1. 分步调试策略

**第一步：调试内环（角速度控制）**
```python
# 将外环增益设为很小
pid_gains.roll_angle_kp = 0.1
pid_gains.pitch_angle_kp = 0.1
# 重点调整内环参数
pid_gains.roll_rate_kp = 50.0  # 逐步增加
```

**第二步：调试外环（角度控制）**
```python
# 内环稳定后，调整外环
pid_gains.roll_angle_kp = 2.0   # 逐步增加
pid_gains.roll_angle_kd = 0.5   # 添加微分
```

### 2. 参数调整指南

| 参数 | 作用 | 调整建议 | 典型范围 |
|------|------|----------|----------|
| 角度P | 角度响应速度 | 从1.0开始增加 | 1.0-5.0 |
| 角度I | 消除角度偏差 | 通常为0或很小 | 0-0.2 |
| 角度D | 减少角度超调 | P的20-30% | 0.2-1.0 |
| 速率P | 速率响应速度 | 从20开始增加 | 20-100 |
| 速率I | 消除速率偏差 | 小心使用 | 0-2.0 |
| 速率D | 抑制高频震荡 | P的10-20% | 2-15 |

### 3. 常见问题处理

**震荡问题**：
- 高频震荡：减小速率环D增益
- 低频震荡：减小角度环P增益
- 持续震荡：检查I增益是否过大

**响应慢**：
- 增加P增益（先内环后外环）
- 适当增加D增益提供阻尼
- 检查输出限制是否太小

**稳态误差**：
- 适当增加I增益
- 确保积分限幅合理
- 检查是否有机械偏差

## 高级功能

### 1. 批量参数测试

```python
# 测试不同参数组合
from pid_params import DEFAULT_PID_GAINS, MY_PID_GAINS
import copy

# 创建参数变体
test_gains = copy.deepcopy(MY_PID_GAINS)
results = []

for kp in [1.5, 2.0, 2.5, 3.0]:
    test_gains.roll_angle_kp = kp
    test_gains.pitch_angle_kp = kp
    # 运行短时仿真并记录性能
    # ...
```

### 2. 参数优化建议

1. **基线建立**：先用DEFAULT_PID_GAINS建立基线
2. **单轴调试**：一次只调一个轴（Roll/Pitch/Yaw）
3. **记录日志**：保存每次调试的参数和结果
4. **安全边界**：设置参数的合理范围

### 3. 与实际飞行对比

仿真调试完成后，实际飞行时：
1. 使用50%的仿真增益作为起点
2. 在安全环境逐步增加
3. 注意仿真未考虑的因素：
   - 电机延迟和不一致性
   - 机架弹性和振动
   - 传感器噪声和延迟

## 扩展开发

### 添加新的控制模式

```python
# 在controller_unified.py中添加
def position_control(self, pos_sp, pos_meas, vel_meas):
    # 位置控制逻辑
    pass
```

### 自定义飞行任务

```python
# 在仿真循环中定义复杂任务
if t < 5.0:
    # 悬停
elif t < 10.0:
    # 画圆
    radius = 1.0
    omega = 0.5
    x_sp = radius * np.cos(omega * t)
    y_sp = radius * np.sin(omega * t)
```

## 故障排查

1. **仿真不稳定**：检查dt是否太大（应≤0.005）
2. **电机饱和**：检查推力限制和PWM范围
3. **参数不生效**：确认使用的是MY_PID_GAINS

## 更新日志

- v2.1.0：独立PID参数管理，简化使用流程
- v2.0.0：统一X型布局，与Keil工程兼容
- v1.0.0：初始版本（已弃用的+型布局）