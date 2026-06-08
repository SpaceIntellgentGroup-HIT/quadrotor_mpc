# 启动指令速查 (Startup Commands)

以下是从原项目中提取的常用启动和操作指令：

## 仿真操作
```bash
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch px4 posix_sitl.launch
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
python3 odom.py
```

## 真机操作
首先测定质量和悬停推力。
然后启动动捕、MAVROS和转发+滤波节点：
```bash
roslaunch vrpn_client_ros sample.launch server:=192.168.0.30
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS0:921600
python3 src/quadrotor_mpc/vrpn_pose_to_odom.py
```

## 核心控制与轨迹生成 (仿真/真机通用)
```bash
roslaunch uav_mpc run_control.launch
roslaunch traj_tools example1.launch
```

## 数据记录与分析
录制 ROS bag：
```bash
bash src/quadrotor_mpc/rosbag_record.sh
```

播放 ROS bag 并可视化：
```bash
rosbag play hover_debug_20260601_151022.bag
rviz -d src/quadrotor_mpc_-main/bag_anal_v0.rviz
```

跑 Rviz 需要发布坐标变换：
```bash
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world map
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link
```