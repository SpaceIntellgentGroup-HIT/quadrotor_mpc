# 四旋翼无人机 MPC 控制系统 (Quadrotor MPC)

本项目是一个基于 ROS1 的功能包，用于实现室内动捕四旋翼无人机的模型预测控制 (MPC)。系统集成了高精度定位、实时状态估计、轨迹规划以及 MPC 核心控制，实现了无人机的高精度轨迹跟踪。

## 1. 软件环境搭建

本项目的核心基于 ROS1 开发，并依赖以下组件进行控制与求解：
*   **ROS 1**：负责各个节点间的通信与核心框架搭建。
*   **ACADO Toolkit & qpOASES**：用于非线性 MPC (NMPC) 的代码生成与快速求解。
*   **MAVROS**：用于与底层基于 STM32 的 PX4 飞控进行串行通信。
*   **PX4 SITL** (可选)：用于在 Gazebo 中进行软件在环仿真及算法验证。
*   **VRPN Client ROS**：用于接收地面动作捕捉系统（MoCap）的定位数据。

## 2. ROS 组件启动方法

### 仿真环境启动流程
1. 启动 PX4 SITL 及 Gazebo 仿真环境：
```bash
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch px4 posix_sitl.launch
```
2. 启动 MAVROS 并连接到仿真飞控：
```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
```
3. 运行状态估计与控制节点：
```bash
python3 odom.py
roslaunch uav_mpc run_control.launch
roslaunch traj_tools example1.launch
```

### 真机环境启动流程
1. 启动 VRPN 动捕客户端：
```bash
roslaunch vrpn_client_ros sample.launch server:=192.168.0.30
```
2. 启动 MAVROS 连接物理飞控：
```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS0:921600
```
3. 运行状态估计与控制节点：
```bash
python3 src/quadrotor_mpc/vrpn_pose_to_odom.py
roslaunch uav_mpc run_control.launch
roslaunch traj_tools example1.launch
```

> **提示**：更多数据记录、回放可视化的启动指令请参考 [README_startup.md](README_startup.md)。

## 3. 代码大致框架

系统基于 ROS 节点的形式开发，主要分为以下几个核心模块与节点：

*   **代码生成 (`mpc_test.cpp`)**：作为离线生成器，利用 ACADO Toolkit 生成 NMPC 的 C 代码并存放到 `quadrotor_mpc_codegen` 目录中。
*   **动捕数据接入 (`vrpn_client_ros`)**：接收地面站发来的无人机绝对位置数据。
*   **状态估计 (`vrpn_pose_to_odom.py` / `pose_to_odom`)**：对动捕数据进行滤波处理，获得高质量的位置和速度估计，因为 MPC 对速度反馈的准确性非常敏感。
*   **轨迹生成 (`traj_tools_node`)**：采用 Min-Jerk / Min-Snap 轨迹生成算法。硬编码航点后进行多项式轨迹插值，并作为参考轨迹（`mpc_ref_traj`，例如 21 个点覆盖 2s 的预测范围）发布给控制节点。
*   **核心控制 (`mpc_node` & `mpc_wrapper.cpp`)**：
    *   `mpc_wrapper.cpp` 是在线 ROS 包装器，包含由 ACADO 生成的头文件，负责 MPC 实时求解。
    *   利用微分平坦性 (Differential Flatness)，将参考轨迹的位置、速度、加速度序列转换为无人机的目标姿态序列。
    *   **推力映射在线估计 (ThrustEstimator)**：动态使用 IMU 的 Z 轴加速度与下发油门队列更新推力系数，补偿电压下降导致的静态误差，将 MPC 的期望加速度精确映射为 MAVROS 的归一化推力。
*   **飞行状态机 (FSM)**：内置在 `mpc_node` 中管理飞行阶段，支持 `AUTO_TAKEOFF`（自动起飞） -> `AUTO_HOVER`（稳健悬停） <-> `AUTO_TRACKING`（轨迹跟踪）的状态流转。

---

> **深入了解**：关于室内动捕硬件的布置细节、推力模型延迟补偿原理、可视化调试（RViz / PlotJuggler）方法及经验总结（踩坑记录），请参考 [README_hardware_and_tips.md](README_hardware_and_tips.md)。