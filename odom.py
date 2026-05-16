#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose

class ModelStatesToOdometry:
    """
    订阅 /gazebo/model_states，将指定模型的状态转换为 Odometry 消息发布
    """
    def __init__(self):
        # 节点参数：机器人模型名称（默认为 'robot'）
        self.model_name = rospy.get_param("~model_name", "iris")
        # 发布的话题名称（默认为 '/odom'）
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        # 坐标系设置
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.child_frame = rospy.get_param("~child_frame", "base_footprint")

        # 创建发布器
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)

        # 创建订阅器
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)

        rospy.loginfo("ModelStatesToOdometry 节点已启动")
        rospy.loginfo("  目标模型: %s", self.model_name)
        rospy.loginfo("  发布话题: %s", self.odom_topic)

    def model_states_callback(self, msg):
        """
        回调函数：处理 ModelStates 消息，转换并发布 Odometry
        :param msg: gazebo_msgs/ModelStates 消息
        """
        # 在模型名称列表中查找目标模型
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            # 如果模型不存在，仅输出一次警告（避免刷屏）
            rospy.logwarn_throttle(5, "模型 '%s' 未在 /gazebo/model_states 中找到", self.model_name)
            return

        # 获取对应模型的位姿和速度
        pose = msg.pose[idx]
        twist = msg.twist[idx]

        # 创建 Odometry 消息
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()  # 或使用 msg.header.stamp（若存在）
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.child_frame

        # 填充位姿
        odom_msg.pose.pose = pose
        # 可选的协方差（如果 Gazebo 模型状态没有协方差，这里保留默认零矩阵）
        # odom_msg.pose.covariance 默认为全 0，可根据需要设置

        # 填充速度（线速度和角速度）
        odom_msg.twist.twist = twist

        # 发布
        self.odom_pub.publish(odom_msg)


if __name__ == "__main__":
    rospy.init_node("model_states_to_odometry", anonymous=False)
    try:
        node = ModelStatesToOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass