# PurePath Controller

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++17](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)
[![ROS2](https://img.shields.io/badge/ROS2-Visualization%20Only-orange)](https://docs.ros.org/)

一个基于平滑控制律（Smooth Control Law）的纯净C++机器人局部路径跟踪控制器，零ROS依赖，可移植到任何嵌入式平台。

## 目录

- [项目特点](#项目特点)
- [原理介绍](#原理介绍)
- [功能特性](#功能特性)
- [系统架构](#系统架构)
- [文件结构](#文件结构)
- [安装与配置](#安装与配置)
- [使用方法](#使用方法)
- [参数说明](#参数说明)
- [未来改进方向](#未来改进方向)
- [许可证](#许可证)

---

## 项目特点

### 纯净C++实现

本项目核心控制器采用纯净C++17实现，无任何ROS依赖：

- `graceful_controller_standalone.hpp` - 头文件，包含所有接口定义
- `graceful_controller_standalone.cpp` - 实现文件，零外部依赖

控制器仅依赖标准库（`<vector>`, `<cmath>`, `<memory>`等），可轻松移植到：
- 嵌入式Linux系统
- STM32 / ESP32 等微控制器
- 裸机环境（Bare Metal）
- 其他机器人中间件（ROS1, ROS2, Cyber RT等）

### ROS2仅用于可视化测试

`ros2_ws/` 目录下的ROS2包装器仅用于：
- 在Gazebo/RViz中可视化测试控制器效果
- 提供激光雷达数据输入示例
- 演示如何将控制器集成到ROS2系统

##生产环境使用时，可直接移植核心库到目标平台。

---

## 原理介绍

### 平滑控制律 (Smooth Control Law)

本控制器基于 Kanayama 等人提出的平滑控制律，通过将目标点转换到机器人坐标系下的极坐标表示，计算曲率和速度指令，并对其进行了优化。

#### 核心数学公式

**1. 极坐标转换 (Egocentric Polar Coordinates)**

将目标点从世界坐标系转换到机器人坐标系：

```
r = √(dx² + dy²)                    // 距离
φ = θ_target - atan2(dy, dx)        // 目标方向与视线夹角  
δ = θ_current - atan2(dy, dx)       // 当前方向与视线夹角
```

**2. 曲率计算 (Curvature Calculation)**

```
κ = -[k_δ(δ - atan(-k_φ·φ)) + (1 + k_φ/(1+(k_φ·φ)²))·sin(δ)] / r
```

其中：
- `k_φ`: 角度误差增益
- `k_δ`: 方向误差增益
- `r`: 机器人到目标点的距离
- `φ`: 目标方向与视线方向的夹角
- `δ`: 当前方向与视线方向的夹角

**3. 速度调节 (Velocity Scaling)**

根据曲率动态调整线速度，实现平滑运动：

```
v = v_max / (1 + β·|κ|^λ)
```

其中：
- `β`: 曲率影响系数
- `λ`: 曲率指数因子

**4. 接近减速 (Slowdown Near Goal)**

当接近目标点时自动减速：

```
if r < slowdown_radius:
    v = v_max × (r / slowdown_radius)
```

### 前视距离 (Lookahead Distance)

控制器使用动态前视距离来选择路径上的跟踪目标点：

```
lookahead = clamp(dist_to_goal, min_lookahead, max_lookahead)
```

- **最小前视距离**: 保证控制稳定性
- **最大前视距离**: 限制跟踪精度

---

## 功能特性

### 核心功能

- [1] **平滑路径跟踪**: 基于曲率的速度调节，实现平滑运动
- [2] **动态前视距离**: 自适应调整跟踪目标点
- [3] **实时碰撞检测**: 基于激光雷达的障碍物检测与避障
- [4] **初始旋转对齐**: 运动前自动调整朝向
- [5] **终点精确定位**: 到达目标点后精确调整姿态
- [6] **速度限制管理**: 支持动态速度限制设置
- [7] **轨迹仿真**: 控制前进行轨迹碰撞检测
- [8] **零ROS依赖**: 核心库仅使用C++标准库

### 碰撞检测

```cpp
class LaserCollisionChecker : public ICollisionChecker {
    // 基于激光雷达的碰撞检测
    // 支持机器人 footprint 碰撞检测
    // 可配置安全距离和检测半径
};
```

### 支持的机器人模型

- 差速驱动机器人 (Differential Drive)
- 支持自定义机器人 footprint

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    Graceful Controller                       │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐ │
│  │  SmoothControl  │  │ ParameterHandler│  │ ICollision  │ │
│  │     Law         │  │                 │  │  Checker    │ │
│  └────────┬────────┘  └─────────────────┘  └──────┬──────┘ │
│           │                                        │        │
│           ▼                                        ▼        │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              GracefulController                      │   │
│  │  - 路径处理    - 速度计算    - 碰撞检测    - 轨迹仿真 │   │
│  └─────────────────────────────────────────────────────┘   │
│                            │                                │
└────────────────────────────┼────────────────────────────────┘
                             │
              ┌──────────────┴──────────────┐
              ▼                             ▼
    ┌─────────────────────┐    ┌─────────────────────┐
    │   Standalone Lib    │    │    ROS2 Wrapper     │
    │   (C++ Library)     │    │   (Visualization)   │
    │   Zero Dependencies │    │   Optional          │
    └─────────────────────┘    └─────────────────────┘
```

### 类结构

| 类名 | 功能描述 |
|------|----------|
| `SmoothControlLaw` | 核心控制律实现，计算曲率和速度 |
| `GracefulController` | 主控制器，整合路径跟踪、碰撞检测 |
| `ParameterHandler` | 参数管理，支持动态参数更新 |
| `ICollisionChecker` | 碰撞检测接口 |
| `LaserCollisionChecker` | 激光雷达碰撞检测实现 |

---

## 文件结构

```
Graceful_controller_c++/
├── graceful_controller_standalone.hpp    # 核心头文件（无ROS依赖）
├── graceful_controller_standalone.cpp    # 核心实现（无ROS依赖）
├── CMakeLists.txt                        # 独立库编译配置
├── config/
│   ├── controller_params.yaml            # 参数配置文件
│   └── graceful_test.rviz                # RViz配置文件
└── ros2_ws/                              # ROS2可视化测试（可选）
    └── src/graceful_controller_ros2/
        ├── src/graceful_controller_ros2.cpp
        ├── launch/controller.launch.py
        └── config/controller_params.yaml
```

---

## 安装与配置

### 环境要求

**核心库（无ROS依赖）:**
- C++17 兼容编译器 (GCC 7+, Clang 5+, MSVC 2017+)
- CMake 3.10+
- 仅依赖C++标准库

**ROS2可视化测试（可选）:**
- Ubuntu 20.04+ / Windows 10+
- ROS2 Foxy / Humble / Rolling

### 独立库编译（推荐）

```bash
# 克隆仓库
git clone https://github.com/Robot-Nav/A-PurePath-Controller.git
cd A-PurePath-Controller

# 创建构建目录
mkdir build && cd build

# 编译
cmake ..
make

# 安装（可选）
sudo make install
```

### 集成到你的项目

**方法1: 直接包含源文件**

```cmake
add_executable(your_robot_node
    your_main.cpp
    graceful_controller_standalone.cpp  # 直接包含
)
target_include_directories(your_robot_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
```

**方法2: 作为静态库**

```cmake
add_subdirectory(graceful_controller)
target_link_libraries(your_robot_node
    graceful_controller_standalone
)
```

### ROS2可视化测试（可选）

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# 启动Gazebo仿真
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 启动控制器
ros2 launch graceful_controller_ros2 controller.launch.py

# 发布测试路径
ros2 run graceful_controller_ros2 publish_offline_path.py
```

---

## 使用方法
详见readme步骤


## 参数说明

### 控制参数

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `min_lookahead` | 0.25 | 最小前视距离 (m) |
| `max_lookahead` | 1.0 | 最大前视距离 (m) |
| `k_phi` | 2.0 | 角度误差增益 |
| `k_delta` | 1.0 | 方向误差增益 |
| `beta` | 0.4 | 曲率影响系数 |
| `lambda` | 2.0 | 曲率指数因子 (≥1) |

### 速度参数

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `v_linear_min` | 0.1 | 最小线速度 (m/s) |
| `v_linear_max` | 0.5 | 最大线速度 (m/s) |
| `v_angular_max` | 1.0 | 最大角速度 (rad/s) |
| `v_angular_min_in_place` | 0.25 | 原地旋转最小角速度 (rad/s) |

### 行为参数

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `slowdown_radius` | 1.5 | 减速半径 (m) |
| `initial_rotation` | true | 启用初始旋转对齐 |
| `initial_rotation_tolerance` | 0.75 | 初始旋转容差 (rad) |
| `prefer_final_rotation` | true | 终点优先旋转到目标方向 |
| `rotation_scaling_factor` | 0.5 | 旋转速度缩放因子 |
| `allow_backward` | false | 允许后退运动 |

### 碰撞检测参数

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `use_collision_detection` | true | 启用碰撞检测 |
| `collision_check_radius` | 0.5 | 碰撞检测半径 (m) |
| `safety_distance` | 0.3 | 安全距离 (m) |
| `in_place_collision_resolution` | 0.1 | 原地旋转碰撞检测分辨率 (rad) |

### 机器人 footprint

```cpp
// C++代码中设置
Footprint footprint = {
    {0.3, 0.3},   // 右前
    {-0.3, 0.3},  // 左前
    {-0.3, -0.3}, // 左后
    {0.3, -0.3}   // 右后
};
controller.setRobotFootprint(footprint);
```

---

## 未来改进方向

### 1. 障碍物跟随功能 (Obstacle Following)

**目标**: 实现沿墙行走或障碍物边界跟踪功能

**实现思路**:
```cpp
class ObstacleFollower {
public:
    // 检测障碍物边界
    bool detectObstacleBoundary(const LaserScan& scan);
    
    // 计算沿墙速度指令
    Twist computeFollowVelocity(const Pose2D& current_pose, 
                                 const LaserScan& scan,
                                 double follow_distance,
                                 FollowSide side);  // LEFT or RIGHT
    
    // 切换跟随方向
    void switchFollowSide(FollowSide side);
    
private:
    // 提取障碍物边界点
    std::vector<Point2D> extractBoundaryPoints(const LaserScan& scan);
    
    // 计算与障碍物的最近距离
    double computeDistanceToObstacle(const LaserScan& scan);
};
```

**应用场景**:
- 狭窄通道通过
- 未知环境探索
- 紧急避障后的路径恢复

### 2. 动态障碍物预测

**目标**: 预测移动障碍物的轨迹，提前规划避障路径

**实现思路**:
- 使用卡尔曼滤波跟踪动态障碍物
- 预测障碍物未来位置
- 在轨迹仿真中考虑动态障碍物

### 3. 自适应参数调整

**目标**: 根据环境复杂度动态调整控制参数

**实现思路**:
```cpp
class AdaptiveParameterTuner {
public:
    void updateParameters(const EnvironmentContext& context);
    
private:
    // 根据路径曲率调整 lookahead
    double adaptLookahead(double path_curvature);
    
    // 根据障碍物密度调整速度
    double adaptSpeed(double obstacle_density);
};
```

### 4. 多路径候选评估

**目标**: 生成多条候选轨迹，选择最优路径

**实现思路**:
- 基于 DWA (Dynamic Window Approach) 生成多组速度指令
- 评估每条轨迹的安全性、平滑性、效率
- 选择最优轨迹执行


## 调试与故障排除

### 常见问题

**1. 机器人运动不平稳**
- 检查 `beta` 和 `lambda` 参数
- 适当减小 `v_linear_max`
- 增大 `min_lookahead`

**2. 跟踪精度不足**
- 减小 `max_lookahead`
- 增大 `k_phi` 和 `k_delta`
- 检查路径发布频率

**3. 碰撞检测误报**
- 调整 `safety_distance`
- 检查 laser scan 数据质量
- 验证 footprint 配置

### 日志输出

控制器内置日志输出，可在控制台查看运行状态：

```
[SmoothControlLaw] ego: r=1.234, phi=0.123, delta=0.045, curvature=0.678, v=0.456, w=0.234
[GracefulController] lookahead=0.8, closest_idx=5, target=(0.8,0.1), dist_to_target=0.8, dist_to_goal=5.2
```

---

## 引用

如果您在研究中使用了本项目，请引用：

```bibtex
@software{purepath_controller,
  title = {PurePath Controller: A Pure C++ Path Tracking Controller for Mobile Robots},
  author = {Robot Nav},
  year = {2026},
  url = {https://github.com/Robot-Nav/A-PurePath-Controller}
}
```

## 参考文献

1. Kanayama, Y., Kimura, Y., Miyazaki, F., & Noguchi, T. (1990). A stable tracking control method for an autonomous mobile robot. Proceedings of the IEEE International Conference on Robotics and Automation.

2. Park, J., & Kuipers, B. (2011). A smooth control law for graceful motion of differential wheeled mobile robots in 2D environment. Proceedings of the IEEE International Conference on Robotics and Automation.

## 许可证

本项目采用 [MIT License](LICENSE) 开源许可证。

```
MIT License

Copyright (c) 2026 Graceful Controller Contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
```

---

## 贡献

欢迎提交 Issue 和 Pull Request！

### 开发流程

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/amazing-feature`)
3. 提交更改 (`git commit -m 'Add amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 创建 Pull Request

### 代码规范

- 遵循 Google C++ Style Guide
- 所有代码必须通过 clang-format 格式化
- 提交前运行测试

---
**Star 这个项目如果它对您有帮助！**

