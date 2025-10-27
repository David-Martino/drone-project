#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('cf3d_controller')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('look_ahead', 0)   # index offset
        self.declare_parameter('tol', 0.12)       # waypoint radius
        self.declare_parameter('kp_xy', 0.6)
        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('vmax_xy', 0.6)
        self.declare_parameter('vmax_z',  0.5)
        self.declare_parameter('odom_topic', '/crazyflie/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.look = int(self.get_parameter('look_ahead').value)
        self.tol = float(self.get_parameter('tol').value)
        self.kp_xy = float(self.get_parameter('kp_xy').value)
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.vmax_xy = float(self.get_parameter('vmax_xy').value)
        self.vmax_z  = float(self.get_parameter('vmax_z').value)
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        self.sub_path = self.create_subscription(Path, '/cf3d/path', self.on_path, 10)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.path_pts = None
        self.cur = None
        self.idx = 0
        self.timer = self.create_timer(0.05, self.step)

    def on_path(self, msg: Path):
        self.path_pts = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses], dtype=np.float32)
        self.idx = 0

    def on_odom(self, msg: Odometry):
        self.cur = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z], dtype=np.float32)

    def step(self):
        if self.path_pts is None or self.cur is None:
            return
        if self.idx >= len(self.path_pts):
            self.pub_cmd.publish(Twist())  # stop
            return
        # advance if inside tolerance
        while self.idx < len(self.path_pts) and np.linalg.norm(self.cur - self.path_pts[self.idx]) < self.tol:
            self.idx += 1
        if self.idx >= len(self.path_pts):
            self.pub_cmd.publish(Twist()); return

        tgt_idx = min(self.idx + self.look, len(self.path_pts)-1)
        tgt = self.path_pts[tgt_idx]
        e = tgt - self.cur
        vx = float(np.clip(self.kp_xy * e[0], -self.vmax_xy, self.vmax_xy))
        vy = float(np.clip(self.kp_xy * e[1], -self.vmax_xy, self.vmax_xy))
        vz = float(np.clip(self.kp_z  * e[2], -self.vmax_z,  self.vmax_z))

        tw = Twist()
        tw.linear.x = vx
        tw.linear.y = vy
        tw.linear.z = vz
        tw.angular.z = 0.0  # yaw-hold for now
        self.pub_cmd.publish(tw)

def main():
    rclpy.init()
    n = WaypointFollower()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
