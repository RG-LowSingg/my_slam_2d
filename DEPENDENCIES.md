# 🤖 SLAM 2D 项目依赖与部署要求文档

本文档详细说明了在 NVIDIA Jetson Orin Nano (或虚拟机测试环境) 上部署和运行本系统所需要的软硬件依赖，以及标准的编译与运行流程。

---

## 1. 核心系统环境

*   **操作系统**: Ubuntu 22.04 (Jammy)
*   **ROS 版本**: ROS 2 Humble (系统级安装于 `/opt/ros/humble`)
*   **硬件平台**: NVIDIA Jetson Orin Nano (8GB) 或同等计算平台

---

## 2. 系统级依赖包 (apt)

本系统底层的传感器驱动（雷达、IMU、串口通信等）依赖于部分 C++ 库和 ROS 2 官方扩展包。必须在**系统原生环境**（非 Conda 环境）下进行全局安装。

请新开一个纯净终端并执行以下命令一键安装：

```bash
sudo apt-get update
sudo apt-get install -y \
    libserial-dev \
    ros-humble-imu-filter-madgwick \
    ros-humble-laser-filters
```

> **注意**：思岚 2D 激光雷达的官方 ROS 2 驱动 `sllidar_ros2` 由于未提供 Humble 版本的 apt 二进制包，已作为源码形式集成到了本工作空间的 `src` 目录下，无需额外通过 apt 安装。

---

## 3. Python 感知环境依赖 (Conda)

本项目中的 3D 深度相机感知节点（`gemini335_perception`）依赖于奥比中光官方的 Python SDK。为了不污染系统环境，我们使用 Conda 进行环境隔离。

### 3.1 创建与激活环境
确保你已安装 `miniforge3` 或 `miniconda3`。
```bash
# 激活名为 gemini_ros2 的环境
conda activate gemini_ros2
```

### 3.2 必须的 Python 依赖
在 `gemini_ros2` 环境下，需要确保安装了以下核心库：
*   `pyorbbecsdk` (奥比中光官方 SDK)
*   `numpy`
*   `opencv-python`

> **机制说明**：我们在 `gemini335_perception` 包的 `setup.cfg` 中配置了 `executable = /usr/bin/env python3`。这保证了即使包是在系统原生环境下编译的，在运行时也会动态抓取当前激活的 Conda Python 解释器。

---

## 4. 标准编译与运行流程 (🔴 关键)

由于 ROS 2 的 C++ 编译系统 (`ament_cmake`) 和 Conda 的 Python 环境存在依赖冲突（Conda 内缺少 `catkin_pkg` 等打包工具），**严禁在激活 Conda 的状态下编译工作空间**。

请严格遵守以下分离式的“纯净编译，Conda 运行”流程：

### 步骤 A：纯净环境编译
```bash
# 1. 彻底退出 Conda 环境（直到行首没有任何环境括号）
conda deactivate

# 2. 进入工作空间
cd ~/slam_2d

# 3. 清理旧缓存（如果在不同环境间切换过，务必执行）
rm -rf build/ install/ log/

# 4. 执行纯净编译
colcon build
```

### 步骤 B：Conda 环境运行
```bash
# 1. 激活包含相机 SDK 的 Python 环境
conda activate gemini_ros2

# 2. 刷新工作空间环境变量 (非常重要！)
cd ~/slam_2d
source install/setup.bash

# 3. 启动整车融合节点
ros2 launch robot_bringup bringup.launch.py serial_port:=/dev/ttyTHS0
```

---

## 5. 硬件外设检查清单

在真机运行前，请确认以下硬件接口已正确连接：
1. **STM32 底盘通信**: 默认连接到 Jetson 的 `/dev/ttyTHS0`（如果发生权限报错，请执行 `sudo chmod 666 /dev/ttyTHS0` 或将当前用户加入 `dialout` 组）。
2. **思岚 2D 激光雷达**: USB 接入，通常为 `/dev/ttyUSB0`（已配置 udev rules 的话通常映射为固定名称）。
3. **奥比中光 Gemini 335 相机**: 必须使用 USB 3.0 数据线接入 Jetson 的 USB 3.0 接口，否则可能因带宽不足导致深度图掉帧。

---
*文档生成于代码架构全面整合及测试通过后，保障了双源避障、统一 TF 树及深度感知系统的稳定运行。*
