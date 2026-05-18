#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class VrpnPoseToOdometry:
    def __init__(self):
        self.pose_topic = rospy.get_param("~pose_topic", "/vrpn_client_node/NMPC0/pose")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.odom_frame = rospy.get_param("~odom_frame", "")
        self.child_frame = rospy.get_param("~child_frame", "base_footprint")

        self.pos_offset_x = float(rospy.get_param("~pos_offset_x", 0.0))
        self.pos_offset_y = float(rospy.get_param("~pos_offset_y", 0.0))
        self.pos_offset_z = float(rospy.get_param("~pos_offset_z", 0.0))

        self.filter_tau = float(rospy.get_param("~filter_tau", 0.01))
        self.velocity_filter_tau = float(rospy.get_param("~velocity_filter_tau", 0.05))
        self.min_dt = float(rospy.get_param("~min_dt", 1.0e-4))
        self.max_dt = float(rospy.get_param("~max_dt", 0.2))
        self.max_velocity = float(rospy.get_param("~max_velocity", 5.0))

        self._last_stamp = None
        self._last_raw_pos = None
        self._filt_pos = None
        self._filt_quat = None
        self._filt_vel = (0.0, 0.0, 0.0)

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback, queue_size=20)

        rospy.loginfo("VrpnPoseToOdometry started")
        rospy.loginfo("  pose_topic: %s", self.pose_topic)
        rospy.loginfo("  odom_topic: %s", self.odom_topic)
        rospy.loginfo("  filter_tau: %.3f", self.filter_tau)
        rospy.loginfo("  velocity_filter_tau: %.3f", self.velocity_filter_tau)

    def pose_callback(self, msg: PoseStamped):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        pos = msg.pose.position
        raw_pos = (
            pos.x + self.pos_offset_x,
            pos.y + self.pos_offset_y,
            pos.z + self.pos_offset_z,
        )

        raw_quat = self._quat_normalize((
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ))

        dt = None
        reset_filter = False
        if self._last_stamp is not None:
            dt = (stamp - self._last_stamp).to_sec()
            if dt < -self.min_dt:
                rospy.logwarn_throttle(
                    1.0,
                    "VRPN timestamp moved backwards by %.6f s; resetting odom filter",
                    -dt,
                )
                reset_filter = True
            elif dt > self.max_dt:
                rospy.logwarn_throttle(
                    1.0,
                    "VRPN timestamp gap %.6f s is too large; resetting odom velocity",
                    dt,
                )
                reset_filter = True

        if self._filt_pos is None or self._filt_quat is None:
            self._filt_pos = raw_pos
            self._filt_quat = raw_quat
            self._filt_vel = (0.0, 0.0, 0.0)
        elif reset_filter:
            self._filt_pos = raw_pos
            self._filt_quat = raw_quat
            self._filt_vel = (0.0, 0.0, 0.0)
        else:
            if dt is not None and dt > self.min_dt:
                alpha = self._filter_alpha(dt, self.filter_tau)
                self._filt_pos = self._lowpass_vector(self._filt_pos, raw_pos, alpha)
                self._filt_quat = self._quat_nlerp(self._filt_quat, raw_quat, alpha)

                if self._last_raw_pos is not None:
                    raw_vel = tuple(
                        (raw_pos[i] - self._last_raw_pos[i]) / dt for i in range(3)
                    )
                    raw_vel = self._limit_vector(raw_vel, self.max_velocity)
                    vel_alpha = self._filter_alpha(dt, self.velocity_filter_tau)
                    self._filt_vel = self._lowpass_vector(
                        self._filt_vel, raw_vel, vel_alpha
                    )

        self._last_stamp = stamp
        self._last_raw_pos = raw_pos

        odom = Odometry()
        odom.header.stamp = stamp
        if self.odom_frame:
            odom.header.frame_id = self.odom_frame
        else:
            odom.header.frame_id = msg.header.frame_id if msg.header.frame_id else "odom"
        odom.child_frame_id = self.child_frame

        odom.pose.pose.position.x = self._filt_pos[0]
        odom.pose.pose.position.y = self._filt_pos[1]
        odom.pose.pose.position.z = self._filt_pos[2]
        odom.pose.pose.orientation.x = self._filt_quat[0]
        odom.pose.pose.orientation.y = self._filt_quat[1]
        odom.pose.pose.orientation.z = self._filt_quat[2]
        odom.pose.pose.orientation.w = self._filt_quat[3]
        odom.twist.twist.linear.x = self._filt_vel[0]
        odom.twist.twist.linear.y = self._filt_vel[1]
        odom.twist.twist.linear.z = self._filt_vel[2]

        self.odom_pub.publish(odom)

    @staticmethod
    def _filter_alpha(dt, tau):
        if dt <= 0.0:
            return 0.0
        if tau <= 0.0:
            return 1.0
        return dt / (tau + dt)

    @staticmethod
    def _lowpass_vector(old_value, new_value, alpha):
        return tuple(
            old_value[i] + alpha * (new_value[i] - old_value[i]) for i in range(3)
        )

    @staticmethod
    def _limit_vector(value, max_norm):
        if max_norm <= 0.0:
            return value
        norm = (value[0] * value[0] + value[1] * value[1] + value[2] * value[2]) ** 0.5
        if norm <= max_norm:
            return value
        scale = max_norm / norm
        return (value[0] * scale, value[1] * scale, value[2] * scale)

    @staticmethod
    def _quat_norm(q):
        return (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]) ** 0.5

    @classmethod
    def _quat_normalize(cls, q):
        n = cls._quat_norm(q)
        if n <= 0.0:
            return (0.0, 0.0, 0.0, 1.0)
        return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)

    @staticmethod
    def _quat_dot(a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]

    @classmethod
    def _quat_nlerp(cls, q0, q1, t):
        if t <= 0.0:
            return q0
        if t >= 1.0:
            return cls._quat_normalize(q1)
        if cls._quat_dot(q0, q1) < 0.0:
            q1 = (-q1[0], -q1[1], -q1[2], -q1[3])
        q = (
            (1.0 - t) * q0[0] + t * q1[0],
            (1.0 - t) * q0[1] + t * q1[1],
            (1.0 - t) * q0[2] + t * q1[2],
            (1.0 - t) * q0[3] + t * q1[3],
        )
        return cls._quat_normalize(q)


def main():
    rospy.init_node("vrpn_pose_to_odometry", anonymous=False)
    VrpnPoseToOdometry()
    rospy.spin()


if __name__ == "__main__":
    main()
