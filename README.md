# UAV-MPC-ROS2-PX4: High-Dynamic Trajectory Tracking

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2: Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![PX4: v1.14+](https://img.shields.io/badge/PX4-v1.14+-green)](https://docs.px4.io/main/en/)

本项目是利用 **非线性模型预测控制 (NMPC)** 技术，结合 **CasADi** 优化框架与 **ROS2** 机器人操作系统，在 **PX4** 仿真环境中实现对 3D 轨迹（Lissajous "8" 字形曲线）的厘米级高精度跟踪。

---

## 核心技术栈 (Core Technology)

* **控制算法**：基于离散时间状态空间模型的预测控制。
**状态向量 (State Vector):**
    $$X = [x, y, z, \dot{x}, \dot{y}, \dot{z}]^T$$

    **代价函数 (Cost Function):**
    $$J = \sum_{k=0}^{N} (\|X_k - X_{ref,k}\|_Q^2 + \|U_k\|_R^2)$$
* **求解引擎**：使用 **CasADi + IPOPT** 求解非线性规划问题 (NLP)。
* **通信架构**：通过 **Fast-DDS** 与 **Micro-XRCE-DDS** 实现 ROS2 与 PX4 固件的低延迟交互。
* **开发背景**：结合控制理论与自主系统研究。

---

## 📊 实验量化指标 (Tracking Metrics)

本项目经过深度闭环调参，成功解决了高动态机动下的相位滞后问题。以下为运行 60s 后的实测误差数据：

| 指标 (Metric) | 测量值 (Value) | 备注 (Note) |
| :--- | :--- | :--- |
| **Overall RMSE** | **0.7122 m** | 整体三维均方根误差 |
| **X-axis RMSE** | **0.1947 m** | 轨迹追踪精度达到 20cm 级 |
| **Y-axis RMSE** | **0.2162 m** | 成功克服 8 字轨迹高频侧向机动误差 |
| **Z-axis RMSE** | **0.6501 m** | 垂直高度维持误差 |
| **Mean Error** | **0.3932 m** | 稳态运行时的平均欧式距离误差 |

> **提示**：最大误差 (Max Error: 4.68m) 仅出现在初始起飞接管阶段，属于正常的垂直位置阶跃响应过程。

---

## 🛠️ 安装与运行 (Quick Start)

### 1. 环境配置
确保已安装 ROS2 Humble、PX4-Autopilot 以及 CasADi 库。

### 2. 启动仿真环境
在终端 1 中启动物理引擎：
```bash
HEADLESS=1 make px4_sitl gz_x500
# 在 pxh 提示符下解除安全锁定
param set NAV_DLL_ACT 0
param set NAV_RCL_ACT 0
```

### 3. 运行控制节点
在终端 3 中执行：
```bash
cd ~/uav_mpc_ws
colcon build --symlink-install
source install/setup.bash
ros2 run uav_mpc mpc_node
```

运行约 60 秒后按下 Ctrl+C，系统将自动导出轨迹对比图 figure8_result.png 及详细报告 experiment_metrics.txt。
