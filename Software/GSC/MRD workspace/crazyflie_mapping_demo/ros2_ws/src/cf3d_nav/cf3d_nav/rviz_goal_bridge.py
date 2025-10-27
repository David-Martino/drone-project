#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped as PWCS
from nav_msgs.msg import Odometry

class RvizGoalBridge(Node):
    def __init__(self):
        super().__init__('rviz_goal_bridge')
        self.declare_parameter('frame_id', 'crazyflie/odom')  # must match your world frame
        self.declare_parameter('odom_topic', '/crazyflie/odom')
        self.declare_parameter('z_mode', 'hold')   # 'hold' or 'fixed'
        self.declare_parameter('fixed_z', 0.8)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.z_mode = self.get_parameter('z_mode').get_parameter_value().string_value
        self.fixed_z = float(self.get_parameter('fixed_z').value)

        self.odom = None
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.on_goal_pose, 10)
        # (optional) if you want to use 2D Pose Estimate tool to set start explicitly:
        self.sub_init = self.create_subscription(PWCS, '/initialpose', self.on_initialpose, 10)

        self.pub_start = self.create_publisher(PoseStamped, '/cf3d/start', 1)
        self.pub_goal  = self.create_publisher(PoseStamped, '/cf3d/goal',  1)

    def on_odom(self, msg: Odometry):
        self.odom = msg

    def on_initialpose(self, msg: PWCS):
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = msg.pose.pose
        # ensure frame/z sane
        if self.z_mode == 'fixed':
            ps.pose.position.z = float(self.fixed_z)
        self.pub_start.publish(ps)
        self.get_logger().info("Start set from /initialpose")

    def on_goal_pose(self, msg: PoseStamped):
        # 1) publish start = current odom (if available)
        if self.odom is not None:
            s = PoseStamped()
            s.header.frame_id = self.frame_id
            s.header.stamp = self.get_clock().now().to_msg()
            s.pose = self.odom.pose.pose
            self.pub_start.publish(s)

        # 2) publish goal = msg.xy + chosen z
        g = PoseStamped()
        g.header.frame_id = self.frame_id
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = msg.pose.position.x
        g.pose.position.y = msg.pose.position.y
        if self.z_mode == 'hold' and self.odom is not None:
            g.pose.position.z = float(self.odom.pose.pose.position.z)
        else:
            g.pose.position.z = float(self.fixed_z)
        g.pose.orientation.w = 1.0
        self.pub_goal.publish(g)
        self.get_logger().info(f"Goal sent to /cf3d/goal at z={g.pose.position.z:.2f}")

def main():
    rclpy.init()
    n = RvizGoalBridge()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
