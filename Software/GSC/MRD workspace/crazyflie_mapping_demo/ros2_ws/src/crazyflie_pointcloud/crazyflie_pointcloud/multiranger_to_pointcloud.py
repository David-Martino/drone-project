#!/usr/bin/env python3
"""
MultiRanger + Odometry -> rolling 3D point cloud (PointCloud2)

- Subscribes to a 4-beam LaserScan (front/right/back/left) published by the MultiRanger
  simulator on /crazyflie/scan (angles typically [-pi, -pi/2, 0, +pi/2]).
- Optionally fuses upward/downward rays from /vertscan (a 2-range LaserScan: [up, down]).
- Integrates over time using /crazyflie/odom pose to place each beam endpoint in world frame.
- Publishes a rolling, voxel-downsampled PointCloud2.

Notes:
- With 1D lidars, you only get sparse rays; move/rotate to paint in 3D.
- Odometry drift will smear points; if you have map→odom TF, switch to TF usage.
"""

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Optional, Tuple
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


@dataclass
class Pose:
    xyz: np.ndarray
    quat: np.ndarray


def quat_to_rotm(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),         2*(xz + wy)],
        [2*(xy + wz),         1 - 2*(xx + zz),     2*(yz - wx)],
        [2*(xz - wy),         2*(yz + wx),         1 - 2*(xx + yy)]
    ], dtype=np.float64)


class MultiRangerPointCloudNode(Node):
    def __init__(self):
        super().__init__('multiranger_pointcloud_node')

        # --- Parameters ---
        self.declare_parameter('scan_topic', '/crazyflie/scan')
        self.declare_parameter('odom_topic', '/crazyflie/odom')
        self.declare_parameter('vertscan_topic', 'crazyflie_real/vertscan')
        self.declare_parameter('output_cloud_topic', '/crazyflie/pointcloud')
        self.declare_parameter('world_frame_id', 'crazyflie/odom')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('decay_seconds', 20.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('max_range_clip', 3.5)
        self.declare_parameter('min_range_clip', 0.01)
        self.declare_parameter('max_blocks', 20000)

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.vertscan_topic = self.get_parameter('vertscan_topic').get_parameter_value().string_value
        self.output_cloud_topic = self.get_parameter('output_cloud_topic').get_parameter_value().string_value
        self.world_frame_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.decay_seconds = float(self.get_parameter('decay_seconds').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.max_range_clip = float(self.get_parameter('max_range_clip').value)
        self.min_range_clip = float(self.get_parameter('min_range_clip').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos)
        self.sub_vert = self.create_subscription(LaserScan, self.vertscan_topic, self.on_vertscan, qos)
        self.pub_cloud = self.create_publisher(PointCloud2, self.output_cloud_topic, 10)

        self.last_pose: Optional[Pose] = None
        self.blocks: Deque[Tuple[float, np.ndarray]] = deque(maxlen=int(self.get_parameter('max_blocks').value))

        self.timer = self.create_timer(1.0 / max(self.publish_rate_hz, 1e-3), self.publish_cloud)
        self.get_logger().info('MultiRangerPointCloudNode started (using /vertscan).')

    # --- Odometry ---
    def on_odom(self, msg: Odometry):
        p = np.array([msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      msg.pose.pose.position.z], dtype=np.float64)
        q = np.array([msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w], dtype=np.float64)
        self.last_pose = Pose(p, q)

    # --- Horizontal scan ---
    def on_scan(self, scan: LaserScan):
        if self.last_pose is None:
            return
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        ranges = np.array(scan.ranges, dtype=np.float64)
        valid = np.isfinite(ranges)
        ranges, angles = ranges[valid], angles[valid]
        if ranges.size == 0:
            return
        ranges = np.clip(ranges, self.min_range_clip, self.max_range_clip)
        dirs_body = np.stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)], axis=1)
        pts_body = (dirs_body.T * ranges).T
        R = quat_to_rotm(self.last_pose.quat)
        t = self.last_pose.xyz
        pts_world = (R @ pts_body.T).T + t
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        self.blocks.append((stamp, pts_world))
        if self.decay_seconds > 0:
            cutoff = stamp - self.decay_seconds
            while self.blocks and self.blocks[0][0] < cutoff:
                self.blocks.popleft()

    # --- Vertical 2-beam scan (/vertscan) ---
    def on_vertscan(self, scan: LaserScan):
        if self.last_pose is None:
            return
        if len(scan.ranges) < 2:
            return
        up_range, down_range = scan.ranges[0], scan.ranges[1]
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        R = quat_to_rotm(self.last_pose.quat)
        t = self.last_pose.xyz
        pts = []
        for rng, zdir in [(up_range, +1.0), (down_range, -1.0)]:
            if math.isfinite(rng):
                r = float(max(self.min_range_clip, min(self.max_range_clip, rng)))
                pt_body = np.array([0.0, 0.0, zdir * r], dtype=np.float64)
                pt_world = (R @ pt_body) + t
                pts.append(pt_world)
        if pts:
            self.blocks.append((stamp, np.vstack(pts)))
        if self.decay_seconds > 0:
            cutoff = stamp - self.decay_seconds
            while self.blocks and self.blocks[0][0] < cutoff:
                self.blocks.popleft()

    # --- Publish downsampled cloud ---
    def publish_cloud(self):
        if not self.blocks:
            return
        pts = np.vstack([blk for (_, blk) in self.blocks])
        if pts.size == 0:
            return
        if self.voxel_size > 0:
            keys = np.floor(pts / self.voxel_size).astype(np.int64)
            _, idx = np.unique(keys, axis=0, return_index=True)
            pts = pts[np.sort(idx)]
        now = self.get_clock().now().to_msg()
        hdr = Header()
        hdr.stamp = now
        hdr.frame_id = self.world_frame_id
        cloud_msg = point_cloud2.create_cloud_xyz32(hdr, pts.tolist())
        self.pub_cloud.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRangerPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
