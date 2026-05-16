# quadrotor_mpc_

source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

roslaunch px4 posix_sitl.launch

roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"

python3 odom.py



`roslaunch uav_mpc run_control.launch`

`roslaunch traj_tools example1.launch`
