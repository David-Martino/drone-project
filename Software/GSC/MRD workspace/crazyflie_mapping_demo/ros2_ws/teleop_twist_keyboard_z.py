#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import termios
import tty
import select
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from rcl_interfaces.msg import SetParametersResult

HELP = r"""
========================================
        Mini Drone Teleop (ROS 2)
         “Push to move” controls
========================================
Hold a key to fly. Releasing the key stops publishing quickly.
If no key has been pressed for > 3s, this node stops publishing
both /manual_cmd_vel and /height_direc until the next key press.

FLIGHT COMMANDS:
  i : move forward
  , : move backward
  j : turn LEFT (yaw +)
  l : turn RIGHT (yaw -)
  t : TAKE-OFF  (ascend vertically)
  b : LAND      (descend vertically)

HEIGHT DIRECTION (/height_direc topic):
  r : hold to GO UP      (publishes 2 continuously)
  v : hold to GO DOWN    (publishes 0 continuously)
  (when neither r nor v is pressed → publishes 1, unless inactive >3s)

SPEED ADJUST:
  q / z : ±10% linear & angular
  w / x : ±10% linear only
  e / c : ±10% angular only

OTHER:
  SPACE or s : cease publishing (no message sent)
  k          : immediately publish zero x/y on /manual_cmd_vel, then cease
  Ctrl+C     : quit

Params:
- inactivity_stop_s (float, default 3.0): stop all publishing after this
  many seconds without ANY key press.
- hold_timeout_ms: key “hold” grace period for continuous publishing
- repeat_rate_hz, max_linear, max_angular, topics, etc.
========================================
"""

def get_key(timeout=0.02):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return ''

def save_term():
    return termios.tcgetattr(sys.stdin)

def set_raw():
    tty.setraw(sys.stdin.fileno())

def restore_term(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

class PushToMoveTeleop(Node):
    def __init__(self):
        super().__init__('push_to_move_teleop')

        self.declare_parameter('cmd_vel_topic', '/manual_cmd_vel')
        self.declare_parameter('height_direc_topic', '/height_direc')
        self.declare_parameter('repeat_rate_hz', 30.0)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('hold_timeout_ms', 150.0)
        self.declare_parameter('inactivity_stop_s', 5.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.height_topic  = self.get_parameter('height_direc_topic').get_parameter_value().string_value
        self.repeat_rate   = float(self.get_parameter('repeat_rate_hz').value)
        self.max_lin       = float(self.get_parameter('max_linear').value)
        self.max_ang       = float(self.get_parameter('max_angular').value)
        self.hold_timeout  = float(self.get_parameter('hold_timeout_ms').value) / 1000.0
        self.inactivity_stop = float(self.get_parameter('inactivity_stop_s').value)

        self.pub_twist  = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_height = self.create_publisher(Int32, self.height_topic, 10)

        # Active states
        self.motion_active = False
        self.z_active = False
        self.height_up_active = False
        self.height_down_active = False

        self.last_motion_time = 0.0
        self.last_z_time = 0.0
        self.last_height_time = 0.0
        self.last_key_time = time.monotonic()

        # State values
        self._vx = 0.0
        self._yaw = 0.0
        self._vz = 0.0

        self.timer = self.create_timer(1.0 / self.repeat_rate, self._on_timer)
        self.add_on_set_parameters_callback(self._on_reconfigure)

        self.get_logger().info(HELP)
        self._print_speeds()

    def _on_reconfigure(self, params):
        for p in params:
            if p.name == 'repeat_rate_hz':
                self.repeat_rate = float(p.value)
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.repeat_rate, self._on_timer)
            elif p.name == 'max_linear':
                self.max_lin = float(p.value); self._print_speeds()
            elif p.name == 'max_angular':
                self.max_ang = float(p.value); self._print_speeds()
            elif p.name == 'hold_timeout_ms':
                self.hold_timeout = float(p.value) / 1000.0
            elif p.name == 'inactivity_stop_s':
                self.inactivity_stop = float(p.value)
        return SetParametersResult(successful=True)

    def _print_speeds(self):
        self.get_logger().info(
            f"Max linear: {self.max_lin:.3f} m/s | Max angular: {self.max_ang:.3f} rad/s"
        )

    def _publish_zero_xy_once(self):
        """Publish a single Twist with zero x and zero yaw (z left at 0)."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        # Do not set linear.z here to avoid unintended vertical change
        self.pub_twist.publish(msg)

    def handle_key(self, key: str):
        # Any key press resets inactivity timer
        self.last_key_time = time.monotonic()

        # speed scaling
        if key in ('q','z','w','x','e','c'):
            if   key == 'q': self.max_lin *= 1.1; self.max_ang *= 1.1
            elif key == 'z': self.max_lin *= 0.9; self.max_ang *= 0.9
            elif key == 'w': self.max_lin *= 1.1
            elif key == 'x': self.max_lin *= 0.9
            elif key == 'e': self.max_ang *= 1.1
            elif key == 'c': self.max_ang *= 0.9
            self._print_speeds()
            return

        # Immediate stop behaviors
        if key in (' ', 's'):
            # cease publishing; send nothing
            self.motion_active = False
            self.z_active = False
            self.height_up_active = False
            self.height_down_active = False
            return

        if key == 'k':
            # publish zero x/y once, then cease publishing
            self._publish_zero_xy_once()
            self.motion_active = False
            self.z_active = False
            self.height_up_active = False
            self.height_down_active = False
            return

        # Forward/back/turn
        if key == 'i':
            self._vx = +self.max_lin; self._yaw = 0.0
            self.motion_active=True; self.last_motion_time=time.monotonic(); return
        if key == ',':
            self._vx = -self.max_lin; self._yaw = 0.0
            self.motion_active=True; self.last_motion_time=time.monotonic(); return
        if key == 'j':
            self._vx = 0.0; self._yaw = +self.max_ang
            self.motion_active=True; self.last_motion_time=time.monotonic(); return
        if key == 'l':
            self._vx = 0.0; self._yaw = -self.max_ang
            self.motion_active=True; self.last_motion_time=time.monotonic(); return

        # Vertical motion (take-off / land)
        if key == 't':
            self._vz = +self.max_lin; self.z_active=True; self.last_z_time=time.monotonic(); return
        if key == 'b':
            self._vz = -self.max_lin; self.z_active=True; self.last_z_time=time.monotonic(); return

        # Height direction continuous keys
        if key == 'r':
            self.height_up_active=True; self.height_down_active=False; self.last_height_time=time.monotonic(); return
        if key == 'v':
            self.height_down_active=True; self.height_up_active=False; self.last_height_time=time.monotonic(); return

    def _on_timer(self):
        now = time.monotonic()

        # Inactivity gate: if no key pressed for > inactivity_stop, publish nothing
        inactive = (now - self.last_key_time) > self.inactivity_stop
        if inactive:
            # also clear active flags so we don't resume without a new key
            self.motion_active = False
            self.z_active = False
            self.height_up_active = False
            self.height_down_active = False
            return  # publish NOTHING (both topics silent)

        publish_motion = self.motion_active and ((now - self.last_motion_time) <= self.hold_timeout)
        publish_z = self.z_active and ((now - self.last_z_time) <= self.hold_timeout)
        publish_up = self.height_up_active and ((now - self.last_height_time) <= self.hold_timeout)
        publish_down = self.height_down_active and ((now - self.last_height_time) <= self.hold_timeout)

        # --- publish Twist ---
        if publish_motion or publish_z:
            msg = Twist()
            if publish_motion:
                msg.linear.x = self._vx
                msg.angular.z = self._yaw
            if publish_z:
                msg.linear.z = self._vz
            self.pub_twist.publish(msg)
        else:
            self.motion_active = False
            self.z_active = False

        # --- publish height_direc ---
        if publish_up:
            self.pub_height.publish(Int32(data=2))
        elif publish_down:
            self.pub_height.publish(Int32(data=0))
        else:
            # neutral (1) if neither pressed
            self.pub_height.publish(Int32(data=1))

def main():
    settings = save_term()
    rclpy.init()
    node = PushToMoveTeleop()
    try:
        set_raw()
        while rclpy.ok():
            key = get_key(timeout=0.02)
            if key == '\x03': break
            if key: node.handle_key(key)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        restore_term(settings)
        node.get_logger().info("Exiting teleop…")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
