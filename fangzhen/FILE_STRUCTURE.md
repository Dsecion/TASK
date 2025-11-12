# 文件结构说明

## 核心仿真文件（X型布局，与Keil兼容）

### 参数定义
- `unified_params.py` - 统一的四轴飞行器参数定义，包含物理参数和PWM映射

### 动力学与控制
- `dynamics_unified.py` - X型四旋翼动力学模型
- `controller_unified.py` - 级联PID控制器（角度环+角速度环）
- `mixer_x_type.py` - X型布局混控算法，与Keil工程一致

### 工具与转换
- `pid_param_converter.py` - PID参数在Python和Keil之间转换
- `calibration_guide.md` - 详细的机体参数测量指南

## 通用组件

- `pid.py` - PID控制器基类实现
- `sensors.py` - 传感器噪声模拟
- `plots.py` - 数据可视化工具

## 示例程序

- `run_unified_simulation.py` - 完整的仿真示例，展示所有功能
- `quick_start.py` - 简化的快速开始示例

## 文档

- `README.md` - 项目概述
- `README_UNIFIED.md` - 详细使用指南
- `FILE_STRUCTURE.md` - 本文件

## 已删除的文件

以下文件使用了不正确的+型布局，已被删除：
- ~~controller.py~~ - 原始+型控制器
- ~~dynamics.py~~ - 原始+型动力学
- ~~mixer.py~~ - 原始+型混控器
- ~~sim.py~~ - 原始仿真配置
- ~~user_params.py~~ - 原始参数文件
- ~~run_example.py~~ - 原始示例
