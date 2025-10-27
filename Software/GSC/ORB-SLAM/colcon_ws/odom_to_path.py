# odom_to_path.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        self.declare_parameter('odom_topic', '/slam/odometry')
        self.declare_parameter('path_topic', '/mypath')
        self.declare_parameter('stride', 1)          # keep every Nth odom
        self.declare_parameter('max_points', 2000)   # truncate path length
        self.declare_parameter('frame_id', '')       # override frame_id if needed

        odom_topic  = self.get_parameter('odom_topic').get_parameter_value().string_value
        path_topic  = self.get_parameter('path_topic').get_parameter_value().string_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(Odometry, odom_topic, self.cb, qos)
        self.pub = self.create_publisher(Path, path_topic, 10)

        self.path = Path()
        self.count = 0

    def cb(self, odom: Odometry):
        self.count += 1
        if self.count % self.get_parameter('stride').value != 0:
            return

        ps = PoseStamped()
        ps.header = odom.header
        override = self.get_parameter('frame_id').get_parameter_value().string_value
        if override:
            ps.header.frame_id = override
        ps.pose = odom.pose.pose

        # initialise/keep consistent frame_id
        if not self.path.header.frame_id:
            self.path.header.frame_id = ps.header.frame_id
        self.path.header.stamp = ps.header.stamp

        self.path.poses.append(ps)
        max_pts = self.get_parameter('max_points').value
        if max_pts > 0 and len(self.path.poses) > max_pts:
            self.path.poses = self.path.poses[-max_pts:]

        self.pub.publish(self.path)

def main():
    rclpy.init()
    node = OdomToPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
