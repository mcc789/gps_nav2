# gps_nav2
ROS 2 Humble Hawksbill 下基于 Nav2 的 FishBot 室外 GPS 导航仿真包（Python 实现）  
本项目实现了基于鱼香ROS差速驱动机器人 FishBot 在 Gazebo 室外场景下的 GPS 导航，集成双 EKF 融合（编码器+IMU+GPS）实现高精度全局定位，适配 **MPPI 局部控制器** 完成平滑避障，支持经纬度目标点导航。

## 功能特性
- **一键式仿真启动**：单条命令启动 Gazebo 室外场景、机器人模型、Nav2 导航栈、GPS/IMU/激光雷达传感器仿真  
- **GPS 坐标转换**：自动将 WGS84 经纬度转换为局部笛卡尔坐标（UTM 投影），适配 Nav2 导航框架  
- **多传感器融合定位**：基于 `robot_localization` 实现双 EKF 融合（轮式里程计+IMU+GPS），提升定位稳定性  
- **MPPI 局部规划控制**：集成 Nav2 MPPI 控制器（Model Predictive Path Integral），相比传统 DWA 更适配差速驱动机器人，避障更平滑、轨迹跟踪更精准  
- **完整的导航能力**：支持 GPS 目标点下发、全局路径规划（Navfn/STAR）、MPPI 局部避障  
- **可视化调试**：RViz 预设配置，一键显示机器人模型、激光点云、GPS 轨迹、规划路径、MPPI 采样轨迹  

## 环境要求 & 快速运行
```bash
# 基础环境
# 操作系统: Ubuntu 22.04 LTS (Jammy Jellyfish)
# ROS 2 版本: Humble Hawksbill (必须严格匹配，不同版本可能兼容异常)
# Gazebo 版本: 11 (Ubuntu 22.04 默认安装版本)

# 环境验证
# 验证 ROS 2 版本
ros2 --version  # 输出应包含 "humble"
# 验证 Gazebo
gazebo --version  # 输出应包含 "Gazebo 11."

# 快速运行
# 一键启动：Gazebo场景 + FishBot模型 + Nav2 + MPPI控制器 + GPS融合定位
ros2 launch gps_nav2 gps_navigation.launch.py

# 运行预设经纬度导航脚本
ros2 run gps_nav2 gps_goal_publisher
