# 四轴飞行器仿真系统（统一版）

这是一个与Keil工程完全兼容的四轴飞行器仿真系统，采用X型布局，支持PID参数的无缝迁移。

## 特点

- ✅ X型四旋翼布局（与实际硬件一致）
- ✅ 物理单位与PWM控制量自动转换
- ✅ 独立的PID参数配置，方便调试
- ✅ 可直接复制参数到Keil工程
- ✅ 提供多种预设参数（默认/保守/激进）

## 快速开始

### 1. 安装依赖

```bash
pip install numpy matplotlib
```

### 2. 调整PID参数

编辑 `pid_params.py` 文件中的 `MY_PID_GAINS`：

```python
MY_PID_GAINS = UnifiedAttitudeGains(
    # 角度环
    roll_angle_kp=2.5,      # <-- 在这里调整
    roll_angle_ki=0.05,
    roll_angle_kd=0.6,
    # ... 其他参数
)
```

### 3. 运行仿真

```bash
# 完整仿真（20秒，包含多个测试阶段）
python run_unified_simulation.py

# 快速测试（10秒）
python quick_start.py
```

### 4. 复制参数到Keil

仿真结束后会显示格式化的参数，直接复制到 `PID_Config.h`：

```c
#define ANGLE_ROLL_KP  2.5f
#define ANGLE_ROLL_KI  0.05f
// ... 其他参数
```

## 文件说明

### PID参数配置
- `pid_params.py` - **主要配置文件**，包含所有PID参数

### 仿真示例
- `run_unified_simulation.py` - 完整仿真，包含性能评估
- `quick_start.py` - 快速测试脚本

### 核心组件
- `unified_params.py` - 机体参数（质量、臂长等）
- `controller_unified.py` - 控制器实现
- `dynamics_unified.py` - X型动力学模型
- `mixer_x_type.py` - X型混控算法

### 工具
- `pid_param_converter.py` - 生成Keil格式的参数
- `calibration_guide.md` - 机体参数测量指南

## PID调试建议

### 调试顺序
1. 先调内环（角速度环）：设置角度环增益为0，直接给角速度指令
2. 再调外环（角度环）：在内环稳定后调整

### 参数调整技巧
- **P增益**：从小开始，逐步增大直到出现轻微震荡，然后回调20%
- **I增益**：用于消除稳态误差，通常很小（0.01-0.5）
- **D增益**：提供阻尼，减少超调，但对噪声敏感

### 预设参数说明
- **DEFAULT_PID_GAINS**：平衡的默认参数
- **CONSERVATIVE_PID_GAINS**：响应慢但稳定
- **AGGRESSIVE_PID_GAINS**：响应快但可能震荡

## 注意事项

1. 仿真使用的PID参数独立存储在 `pid_params.py`
2. 调试完成后手动复制到Keil工程
3. 首次实飞请使用保守参数
4. 确保电机编号与硬件一致（X型布局）

## 版本信息

- 版本：2.1.0
- 布局：X型四旋翼
- 兼容：Keil工程