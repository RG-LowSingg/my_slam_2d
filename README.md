# SLAM 2D 项目 (SLAM 2D Project)

这是一个基于 ROS 2 Humble 和 NVIDIA Jetson Orin Nano 开发的 2D SLAM 与多传感器融合导航项目。本项目集成了底盘控制、2D 激光雷达、IMU、以及深度相机等传感器模块，旨在提供一个完整且稳定的自主移动机器人底盘建图与感知解决方案。

## 项目结构 (Project Structure)

工作空间 `src` 目录下的核心功能包：

*   **`robot_bringup`**: 机器人的核心启动包，包含所有核心 `launch` 文件、参数配置以及机器人的 URDF/Xacro 模型，负责启动和管理整个系统的节点。
*   **`robot_serial`**: 串口通信节点，负责与底层硬件底盘（如 STM32）进行数据交互，下发速度控制指令并读取底盘实时状态。
*   **`robot_imu`**: IMU 传感器模块，包含数据读取和姿态解算。
*   **`robot_lidar`**: 2D 激光雷达数据处理与过滤节点。
*   **`sllidar_ros2`**: 思岚 (Slamtec) 2D 激光雷达的官方 ROS 2 驱动程序源文件。
*   **`robot_odometry`**: 轮式里程计计算节点，基于底盘编码器反馈，并可结合 IMU 数据计算机器人的高精度位姿。
*   **`gemini335_perception`**: 基于奥比中光 (Orbbec) Gemini 335 深度相机的 3D 视觉感知模块。该模块运行于 Conda Python 环境中，主要用于深度数据处理及双源避障。

## 系统特性 (System Features)

- **原生 ROS 2 架构**: 全面基于 ROS 2 Humble 框架开发。
- **多源传感器集成**: 支持 2D LiDAR建图、3D 深度相机避障、IMU 姿态解算及轮式里程计。
- **主流 SLAM 算法**: 已在依赖中集成了 `slam_toolbox` 和 `cartographer_ros` 的支持，方便直接进行 2D 建图任务。
- **双环境解耦运行**: 创新性地采用“原生系统环境编译，Conda 隔离环境运行”的机制，完美解决了 ROS 2 底层 C++ 依赖与相机 SDK Python 依赖之间的冲突问题。

## 部署与运行 (Deployment & Usage)

本项目对软硬件环境和编译流程有**极其严格的要求**。

请在尝试编译或运行之前，**务必仔细阅读**根目录下的 [DEPENDENCIES.md](DEPENDENCIES.md) 文档，其中详细记录了：

1.  **Apt 系统级依赖包**的安装方法。
2.  **Conda Python 隔离环境**的创建与配置。
3.  **标准编译与运行流程 (🔴 关键)**：绝对禁止在 Conda 激活状态下进行 `colcon build`。
4.  **硬件串口与 USB 设备**权限检查清单。

### 快速启动 (Quick Start)

*(⚠️ 警告：执行前请确保已经按照 `DEPENDENCIES.md` 的步骤完成了系统的纯净编译，且已激活所需的 Conda 环境。)*

```bash
# 1. 激活 Conda 环境
conda activate gemini_ros2

# 2. 进入工作空间并刷新环境变量
cd ~/slam_2d
source install/setup.bash

# 3. 启动整车融合系统 (需根据实际情况指定串口)
ros2 launch robot_bringup bringup.launch.py serial_port:=/dev/ttyTHS0
```

## Verification Plan

### Phase 1: TF 树验证
```bash
# 启动底盘 + 相机
ros2 launch robot_bringup bringup.launch.py serial_port:=/dev/ttyTHS0
# 检查 TF 树完整性
ros2 run tf2_tools view_frames
# 应看到: map → odom → base_link → {laser_frame, imu_link, gemini335_camera_link}
#                                      └→ gemini335_depth_optical_frame
#                                      └→ gemini335_color_optical_frame
```

### Phase 2: 话题清单检查
```bash
ros2 topic list | grep -E "scan|imu|odom|cmd_vel|velocity|gemini"
# 应精确看到 (无冗余):
# /velocity
# /imu/data_raw
# /imu/data
# /odom
# /scan_raw
# /scan
# /gemini335/depth/scan
# /gemini335/imu
# /gemini335/color/image_raw     (仅有订阅者时活跃)
# /gemini335/depth/image_raw     (仅有订阅者时活跃)
# /cmd_vel
```

### Phase 3: SLAM 建图验证
```bash
ros2 launch robot_bringup slam.launch.py
# RViz 中同时显示 /scan (雷达, 360°) 和 /gemini335/depth/scan (相机, 前方约70°)
# 建图结果: 走廊宽度与物理测量偏差 < 5cm，闭环无显著偏移
```

### Phase 4: 双源 Costmap 验证
```bash
ros2 launch robot_bringup navigation.launch.py
# RViz 中观察 Local Costmap:
# - 底盘雷达标记的 360° 障碍物
# - 相机标记的前方上层空间障碍物 (如桌面悬空部分)
# 两者应正确叠加，无 TF 错位
```

### Phase 5: 闯入守卫测试
```bash
# 导航中，在相机前方 <1m 处挥手或放置高于雷达扫描面的障碍物
# 预期行为:
#   1. 机器人平缓停止 (非急刹)
#   2. 终端日志: "Intrusion detected, canceling navigation"
#   3. 障碍物移除后约 0.3s，机器人自动恢复原导航目标
#   4. 终端日志: "Intrusion cleared, resuming navigation to cached goal"
```

### Phase 6: Jetson Orin Nano 性能监控
```bash
tegrastats  # 持续监控 CPU/GPU/内存 使用率
# CPU 总占用应 < 70%
# GPU 占用 ≈ 0% (本阶段)
# 内存 < 4GB
```

### Phase 7: 串口通信健康检查
```bash
# 验证 UART3 双向通信
ros2 topic echo /velocity --once  # 应看到底盘编码器速度
ros2 topic echo /imu/data_raw --once  # 应看到 ICM20948 原始数据
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once
# 底盘应前进约 0.1m/s
```
