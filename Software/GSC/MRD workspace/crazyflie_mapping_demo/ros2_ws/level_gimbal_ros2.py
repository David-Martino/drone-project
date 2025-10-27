#!/usr/bin/env python3
"""
level_gimbal_ros2.py — Keep a link level (roll=0, pitch=0) using ROS 2 + ros_gz_bridge topics.

This node:
  - Subscribes to /world/<world>/pose/info (Pose_V) to read world poses for all entities.
  - Computes yaw from a follow entity (e.g., 'crazyflie/body').
  - Publishes a Pose to /world/<world>/set_pose with the controlled entity's name and position,
    and orientation set to yaw-only (roll=0, pitch=0).

Works with either ros_gz_interfaces OR ros_ign_interfaces (auto-fallback).
"""

import math
import rclpy
from rclpy.node import Node

# Try ros_gz first, then ros_ign (older)
try:
    from ros_gz_interfaces.msg import Pose as GzPoseMsg
    from ros_gz_interfaces.msg import Pose_V as GzPoseVMsg
except ImportError:
    from ros_ign_interfaces.msg import Pose as GzPoseMsg
    from ros_ign_interfaces.msg import Pose_V as GzPoseVMsg


def euler_from_quat(qw, qx, qy, qz):
    # Convert quaternion to roll, pitch, yaw (radians)
    sinr_cosp = 2.0 * (qw*qx + qy*qz)
    cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw*qy - qz*qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi/2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quat_from_yaw(yaw):
    half = 0.5 * yaw
    return (math.cos(half), 0.0, 0.0, math.sin(half))  # (w,x,y,z)


class LevelGimbalNode(Node):
    def __init__(self):
        super().__init__('level_gimbal_ros2')
        self.declare_parameter('world', 'default')
        self.declare_parameter('follow_name', 'crazyflie/body')
        self.declare_parameter('control_name', 'multiranger_link')
        self.declare_parameter('rate_hz', 100.0)

        self.world = self.get_parameter('world').get_parameter_value().string_value
        self.follow_name = self.get_parameter('follow_name').get_parameter_value().string_value
        self.control_name = self.get_parameter('control_name').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value

        self.pose_topic = f"/world/{self.world}/pose/info"
        self.set_pose_topic = f"/world/{self.world}/set_pose"

        self._last_follow_yaw = None
        self._last_control_pose = None

        self._sub = self.create_subscription(GzPoseVMsg, self.pose_topic, self._pose_cb, 10)
        self._pub = self.create_publisher(GzPoseMsg, self.set_pose_topic, 10)
        self._timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self._tick)

        self.get_logger().info(f"[level_gimbal_ros2] world='{self.world}', follow='{self.follow_name}', control='{self.control_name}'")
        self.get_logger().info(f"[level_gimbal_ros2] Subscribing: {self.pose_topic}")
        self.get_logger().info(f"[level_gimbal_ros2] Publishing:  {self.set_pose_topic}")

    def _pose_cb(self, msg: GzPoseVMsg):
        # Update cached yaw of follow entity and current control pose (position/orientation)
        for p in msg.pose:
            name = p.name
            # Messages store orientation as (x,y,z,w)
            qx, qy, qz, qw = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w

            if name == self.follow_name:
                _, _, yaw = euler_from_quat(qw, qx, qy, qz)
                self._last_follow_yaw = yaw

            if name == self.control_name:
                self._last_control_pose = p

    def _tick(self):
        if self._last_follow_yaw is None or self._last_control_pose is None:
            return

        yaw = self._last_follow_yaw
        p = self._last_control_pose

        msg = GzPoseMsg()
        msg.name = self.control_name
        # Keep current position
        msg.position.x = p.position.x
        msg.position.y = p.position.y
        msg.position.z = p.position.z

        # Set yaw-only orientation
        qw, qx, qy, qz = quat_from_yaw(yaw)
        msg.orientation.w = qw
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz

        self._pub.publish(msg)

def main():
    rclpy.init()
    node = LevelGimbalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
