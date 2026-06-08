# quadrotor_mpc_
仿真操作：
`source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default`

`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic`

`roslaunch px4 posix_sitl.launch`

`roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"`

`python3 odom.py`

真机操作：
首先测定质量和悬停推力
然后开动捕，mavros和转发+滤波
`roslaunch vrpn_client_ros sample.launch server:=192.168.0.30`
`roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS0:921600`
`python3 src/quadrotor_mpc/vrpn_pose_to_odom.py`

然后都一样：

`roslaunch uav_mpc run_control.launch`

`roslaunch traj_tools example1.launch`


如何录制rosbag：
`bash src/quadrotor_mpc/rosbag_record.sh`
如何播放rosbag并可视化：
`rosbag play hover_debug_20260601_151022.bag`
`rviz -d src/quadrotor_mpc_-main/bag_anal_v0.rviz`

跑rviz需要发布坐标变换：

`rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world map`

`rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link`