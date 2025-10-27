from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node


class ControlServices(Node):

    def __init__(self):
        super().__init__('control_services')
        self.declare_parameter('hover_height', 0.5)
        self.declare_parameter('robot_prefix', '/crazyflie')
        self.declare_parameter('incoming_twist_topic', '/cmd_vel')
        self.declare_parameter('height_setpoint_topic', '/cmd_height')
        self.declare_parameter('max_ang_z_rate', 0.4)

        hover_height = self.get_parameter('hover_height').value
        robot_prefix = self.get_parameter('robot_prefix').value
        incoming_twist_topic = self.get_parameter('incoming_twist_topic').value
        max_ang_z_rate = self.get_parameter('max_ang_z_rate').value
        height_setpoint_topic = self.get_parameter('height_setpoint_topic').value

        self.publisher_ = self.create_publisher(Twist, robot_prefix + incoming_twist_topic, 10)
        self.subscriber = self.create_subscription(Odometry, robot_prefix + '/odom', self.odometry_callback, 10)
        self.subscriber = self.create_subscription(Twist, incoming_twist_topic, self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.subscriber = self.create_subscription(Float32, height_setpoint_topic, self.height_sp_callback, 10)

        self.takeoff_command = False
        self.current_pose = Odometry().pose.pose
        self.takeoff_height = hover_height
        self.max_ang_z_rate = max_ang_z_rate
        self.is_flying = False
        self.keep_height = False
        self.teleop_cmd = Twist()
        self.cmd_height_sp = None  # latest /cmd_height (Float32)

    def timer_callback(self):
        msg = self.teleop_cmd
        height_command = msg.linear.z
        new_cmd_msg = Twist()

        # If the drone is flying, pass through XY & yaw from teleop/auto (i.e DONT LISTEN TO COMMAND VEL Z IN SIM WHEN ALRADY FLYING)
        if self.is_flying:
            new_cmd_msg.linear.x = msg.linear.x
            new_cmd_msg.linear.y = msg.linear.y
            new_cmd_msg.angular.x = msg.angular.x
            new_cmd_msg.angular.y = msg.angular.y
            new_cmd_msg.angular.z = msg.angular.z

        # If not flying and receiving a velocity height command, takeoff
        if height_command > 0 and not self.is_flying:
            # mirror vel_mux behaviour: lift to /cmd_height if available
            target_h = self.cmd_height_sp if self.cmd_height_sp is not None else self.takeoff_height
            # simple proportional climb until reaching target height
            if self.current_pose.position.z < target_h:
                new_cmd_msg.linear.z = 0.5
            else:
                new_cmd_msg.linear.z = 0.0
                self.teleop_cmd.linear.z = 0.0
                self.is_flying = True
                self.get_logger().info(f'Takeoff completed at target height {target_h:.2f} m')


        # If flying and if the height command is negative, and it is below a certain height
        # then consider it a land
        if height_command < 0 and self.is_flying:
            if self.current_pose.position.z < 0.1:
                new_cmd_msg.linear.z = 0.0
                self.is_flying = False
                self.keep_height = False
                self.get_logger().info('Landing completed')

        # Cap the angular rate command in the z axis
        if abs(msg.angular.z) > self.max_ang_z_rate:
            new_cmd_msg.angular.z = self.max_ang_z_rate * abs(msg.angular.z)/msg.angular.z

        # Height control logic while already flying
        tolerance = 1e-7
        if self.is_flying:
            if abs(height_command) > tolerance:
                # Explicit z-velocity command present -> pass it through (start/stop behaviour)
                new_cmd_msg.linear.z = height_command
                if self.keep_height:
                    self.keep_height = False
            else:
                # No z-vel command -> hold either /cmd_height (preferred) or last hover height
                if self.cmd_height_sp is not None:
                    error = self.cmd_height_sp - self.current_pose.position.z
                    new_cmd_msg.linear.z = error
                else:
                    # fallback to "keep current height" (original behaviour)
                    if not self.keep_height:
                        self.desired_height = self.current_pose.position.z
                        self.keep_height = True
                    error = self.desired_height - self.current_pose.position.z
                    new_cmd_msg.linear.z = error

        self.publisher_.publish(new_cmd_msg)

    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose

    def takeoff_callback(self, request, response):

        self.takeoff_command = True
        response.success = True
        return response

    def cmd_vel_callback(self, msg):
        self.teleop_cmd = msg

    def height_sp_callback(self, msg: Float32):
       # Update the target altitude whenever a new setpoint arrives
        self.cmd_height_sp = float(msg.data)

def main(args=None):
    rclpy.init(args=args)

    control_services = ControlServices()

    rclpy.spin(control_services)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
