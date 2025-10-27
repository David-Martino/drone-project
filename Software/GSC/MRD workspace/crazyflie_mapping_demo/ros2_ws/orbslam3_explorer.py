#!/usr/bin/env python3
# corridor_explorer.py
# ROS2 Humble - Standalone state-machine explorer for 4-beam LaserScan + odom
# Publishes planar velocity to /auto_cmd_vel and height setpoint to /auto_cmd_height

import math
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

def ang_wrap(a):
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def quat_to_yaw(qx, qy, qz, qw):
    """Extract yaw from quaternion."""
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

class State(Enum):
    INTERSECTION = auto()
    GO_STRAIGHT = auto()
    TURN_LEFT = auto()
    TURN_RIGHT = auto()
    GO_BACKWARDS = auto()
    STRAIGHT_THROUGH_CORRIDOR = auto()
    BACKWARDS_THROUGH_CORRIDOR = auto()
    EXITING_INTERSECTION = auto()
    # NEW: forward confirmation approach after front hit primary threshold
    FORWARD_APPROACH_CONFIRM = auto()
    # NEW: 2s straight roll-in (yaw locked) before turning when intersection wasn't from a front wall
    PRETURN_STRAIGHT = auto()

class CorridorExplorer(Node):
    def __init__(self):
        super().__init__('corridor_explorer')

        # ---------------- Parameters ----------------
        self.declare_parameter('scan_topic', '/crazyflie/scan')
        self.declare_parameter('odom_topic', '/crazyflie/odom')
        self.declare_parameter('cmd_vel_topic', '/auto_cmd_vel')
        self.declare_parameter('cmd_height_topic', '/auto_cmd_height')

        self.declare_parameter('default_height', 0.5)         # m
        self.declare_parameter('side_open_thresh', 2.0)

        # Primary front threshold for detecting a wall ahead (YOU will set this to 2.0m)
        self.declare_parameter('intersection_front_thresh', 2.0)  # m (was 0.5 before)

        # NEW: Secondary confirmation stop threshold (keep going until this, unless side opens)
        self.declare_parameter('front_confirm_stop', 1.0)     # m

        self.declare_parameter('corridor_v', 0.3)             # m/s (forward corridor)
        self.declare_parameter('nudge_v', 0.1)                # m/s (forward/back short nudge)

        # Stronger, sign-correct centring control
        self.declare_parameter('yaw_k', 0.6)                  # P-gain for wall centring
        self.declare_parameter('max_yaw_rate', 0.5)           # rad/s clamp for centring

        self.declare_parameter('turn_rate', 0.6)              # rad/s (stationary turn)
        self.declare_parameter('turn_angle_deg', 90.0)        # degrees
        self.declare_parameter('nudge_duration_s', 0.7)       # seconds to publish the post-turn/back/straight nudge
        self.declare_parameter('control_rate_hz', 20.0)       # main loop rate
        # NEW: duration of straight roll-in before turning (no launch arg; param only)
        self.declare_parameter('preturn_straight_s', 4.0)

        # Logging control
        self.declare_parameter('log_ranges_on_state', True)   # log F/L/B/R on state change

        # Fetch params
        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_height_topic = self.get_parameter('cmd_height_topic').value

        self.default_height = float(self.get_parameter('default_height').value)
        self.SIDE_OPEN = float(self.get_parameter('side_open_thresh').value)

        self.FRONT_T = float(self.get_parameter('intersection_front_thresh').value)
        self.FRONT_CONFIRM = float(self.get_parameter('front_confirm_stop').value)

        self.V_CORRIDOR = float(self.get_parameter('corridor_v').value)
        self.V_NUDGE = float(self.get_parameter('nudge_v').value)
        self.K_YAW = float(self.get_parameter('yaw_k').value)
        self.MAX_YAW = float(self.get_parameter('max_yaw_rate').value)
        self.TURN_RATE = float(self.get_parameter('turn_rate').value)
        self.TURN_ANGLE = math.radians(float(self.get_parameter('turn_angle_deg').value))
        self.NUDGE_DT = float(self.get_parameter('nudge_duration_s').value)
        self.CTRL_HZ = float(self.get_parameter('control_rate_hz').value)
        self.PRETURN_DT = float(self.get_parameter('preturn_straight_s').value)

        self.LOG_RANGES_ON_STATE = bool(self.get_parameter('log_ranges_on_state').value)

        # ---------------- I/O ----------------
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, qos)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)

        self.vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.height_pub = self.create_publisher(Float32, self.cmd_height_topic, 10)

        # ---------------- State vars ----------------

        # Ensure these exist before any logging that calls _ranges()
        self.last_scan = None
        self.have_scan_mapping = False

        self.state = State.STRAIGHT_THROUGH_CORRIDOR  # start by moving forward down a corridor
        self._log_state(self.state)

        self.came_from_backwards_corridor = False  # for intersection priority rule
        self._nudge_until = 0.0                    # time until which to keep nudging
        self._exiting_forward = True               # which direction we’re exiting intersection with

        # NEW: preturn straight timer + queued turn state
        self._preturn_until = 0.0
        self._pending_turn_state = None

        # Scan mapping: indices for front/left/back/right beams (computed from angles)
        self.idx_front = None
        self.idx_left = None
        self.idx_back = None
        self.idx_right = None

        # Odom pose
        self.have_odom = False
        self.yaw = 0.0
        self.turn_target_yaw = None  # radians when in TURN_* states

        # Intersection memory (still using snapshot + reason, but no hysteresis/counters)
        self.intersection_snapshot = None   # (front,left,back,right) captured on entry
        self.intersection_reason = None     # 'front_blocked' | 'back_blocked' | 'side_open' | 'unknown'

        # NEW: remember a side opening seen during forward-approach, but delay the turn until confirm distance
        self.pending_side = None            # None | 'left' | 'right'

        # Main control timer
        self.timer = self.create_timer(1.0 / self.CTRL_HZ, self._control_tick)

    # --------------- Subscribers ----------------
    def _on_scan(self, msg: LaserScan):
        self.last_scan = msg
        if not self.have_scan_mapping:
            self._compute_scan_mapping(msg)

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.have_odom = True

    # --------------- Helpers ----------------
    def _compute_scan_mapping(self, scan: LaserScan):
        """Map the four-beam LaserScan to front/left/back/right by angle proximity."""
        n = len(scan.ranges)
        if n < 4 or scan.angle_increment == 0.0:
            self.get_logger().warn("Scan does not look like a 4-beam LaserScan yet; waiting...")
            return

        # Target angles for front, left, back, right in the scan frame
        targets = {
            'front': 0.0,
            'left': math.pi / 2.0,
            'back': math.pi,
            'right': -math.pi / 2.0
        }

        # Build list of (index, angle) for beams
        beams = []
        for i in range(n):
            ang = scan.angle_min + i * scan.angle_increment
            ang = ang_wrap(ang)
            beams.append((i, ang))

        # Greedy assign each target to nearest beam by angular distance
        assigned = {}
        used = set()
        for name, targ in targets.items():
            best_i, best_d = None, 1e9
            for (i, ang) in beams:
                if i in used:
                    continue
                d = abs(ang_wrap(ang - targ))
                if d < best_d:
                    best_d, best_i = d, i
            assigned[name] = best_i
            used.add(best_i)

        self.idx_front = assigned['front']
        self.idx_left = assigned['left']
        self.idx_back = assigned['back']
        self.idx_right = assigned['right']
        self.have_scan_mapping = True

        self.get_logger().info(
            f"Scan mapping set: front={self.idx_front}, left={self.idx_left}, "
            f"back={self.idx_back}, right={self.idx_right}"
        )

    def _ranges(self):
        """Return (front, left, back, right) distances. Returns None if unavailable."""
        scan = getattr(self, 'last_scan', None)
        if (scan is None) or (not getattr(self, 'have_scan_mapping', False)):
            return None
        r = scan.ranges
        try:
            f = r[self.idx_front]
            l = r[self.idx_left]
            b = r[self.idx_back]
            rr = r[self.idx_right]
            return (f, l, b, rr)
        except Exception as e:
            self.get_logger().warn(f"Scan indexing issue: {e}")
            return None


    def _publish_height(self):
        msg = Float32()
        msg.data = float(self.default_height)
        self.height_pub.publish(msg)

    def _send_vel(self, vx=0.0, vy=0.0, wz=0.0):
        # Replace NaN or Inf with zero to prevent invalid commands (the sim nan joint velocity value error)
        for val_name, val in zip(["vx", "vy", "wz"], [vx, vy, wz]):
            if not math.isfinite(val):
                self.get_logger().warn(f"Invalid velocity ({val_name}={val}); publishing 0 instead.")
                if val_name == "vx":
                    vx = 0.0
                elif val_name == "vy":
                    vy = 0.0
                else:
                    wz = 0.0

        """Publish planar velocity (x forward, y sideways, yaw rate)."""
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.linear.z = 0.0  # controller ignores z on this topic
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = float(wz)
        self.vel_pub.publish(t)

    def _log_state(self, s: State):
        if self.LOG_RANGES_ON_STATE:
            rng = self._ranges()
            if rng is not None:
                f, l, b, r = rng
                self.get_logger().info(f"[STATE] {s.name} | LIDARS (m): F={f:.2f} L={l:.2f} B={b:.2f} R={r:.2f}")
                return
        self.get_logger().info(f"[STATE] {s.name}")

    # --------------- State machine core ----------------
    def _control_tick(self):
        # Always publish the current height setpoint
        self._publish_height()

        rng = self._ranges()
        if rng is None or not self.have_odom:
            # Not enough info yet; keep still
            self._send_vel(0.0, 0.0, 0.0)
            return

        front, left, back, right = rng
        now = time.time()

        # ------------ Transition checks & actions per state ------------
        if self.state == State.STRAIGHT_THROUGH_CORRIDOR:
            # Keep moving forward with yaw correction (wall centring).
            wz = self._corridor_yaw_control(left, right, forward=True)
            self._send_vel(self.V_CORRIDOR, 0.0, wz)

            # NEW logic:
            # If front trips the primary threshold, begin forward-approach confirmation
            if front < self.FRONT_T:
                # starting a fresh approach; clear any previous pending side
                self.pending_side = None
                self.state = State.FORWARD_APPROACH_CONFIRM
                self.intersection_reason = 'front_blocked'
                self.intersection_snapshot = (front, left, back, right)
                self._log_state(self.state)
            # Otherwise, classic side-opening to intersection
            elif (left >= self.SIDE_OPEN) or (right >= self.SIDE_OPEN):
                self.state = State.INTERSECTION
                self.came_from_backwards_corridor = False
                self.intersection_reason = 'side_open'
                self.intersection_snapshot = (front, left, back, right)
                self._log_state(self.state)

        elif self.state == State.FORWARD_APPROACH_CONFIRM:
            # Keep moving forward and keep centring, watching for side openings
            wz = self._corridor_yaw_control(left, right, forward=True)
            self._send_vel(self.V_CORRIDOR, 0.0, wz)

            # If a side opens at any time, remember it but DO NOT turn yet
            if (left >= self.SIDE_OPEN) or (right >= self.SIDE_OPEN):
                if (left >= self.SIDE_OPEN) and (right >= self.SIDE_OPEN):
                    self.pending_side = 'right' if (right >= left) else 'left'
                elif left >= self.SIDE_OPEN:
                    self.pending_side = 'left'
                else:
                    self.pending_side = 'right'
                # Optional: lightweight log (not a state change)
                self.get_logger().info(f"[APPROACH] Side opening detected -> pending turn: {self.pending_side}")

            # On reaching the confirmation distance, either turn to the pending side (if any)
            # or proceed to the original backwards flow.
            if front <= self.FRONT_CONFIRM:
                # Brief stop to avoid bumping the wall
                self._send_vel(0.0, 0.0, 0.0)

                if self.pending_side in ('left', 'right'):
                    # Defer-turn now that we've centred at the intersection mouth
                    if self.pending_side == 'left':
                        self.state = State.TURN_LEFT
                        self.turn_target_yaw = ang_wrap(self.yaw + self.TURN_ANGLE)
                    else:
                        self.state = State.TURN_RIGHT
                        self.turn_target_yaw = ang_wrap(self.yaw - self.TURN_ANGLE)
                    self.came_from_backwards_corridor = False
                    # Clear snapshot/reason; this is a deterministic, deferred choice
                    self.intersection_snapshot = None
                    self.intersection_reason = None
                    # Clear pending flag for next time
                    self.pending_side = None
                    self._log_state(self.state)
                else:
                    # No side opening observed during approach -> proceed to backwards flow as before
                    self.state = State.GO_BACKWARDS
                    self._nudge_until = now + self.NUDGE_DT
                    self._exiting_forward = False
                    # Mark that this decision came from forward corridor
                    self.came_from_backwards_corridor = False
                    self._log_state(self.state)

        elif self.state == State.BACKWARDS_THROUGH_CORRIDOR:
            # Move backwards with flipped yaw correction
            wz = self._corridor_yaw_control(left, right, forward=False)
            self._send_vel(-self.V_CORRIDOR, 0.0, wz)

            # Intersection detection from corridor (backwards rules) - immediate
            if (back < self.FRONT_T) or (left >= self.SIDE_OPEN) or (right >= self.SIDE_OPEN):

                self.state = State.INTERSECTION
                self.came_from_backwards_corridor = True
                # reason + snapshot
                if back < self.FRONT_T:
                    self.intersection_reason = 'back_blocked'
                elif (left >= self.SIDE_OPEN) or (right >= self.SIDE_OPEN):
                    self.intersection_reason = 'side_open'
                else:
                    self.intersection_reason = 'unknown'
                self.intersection_snapshot = (front, left, back, right)
                self._log_state(self.state)

        elif self.state == State.INTERSECTION:
            # Decide next state based on priorities (using snapshot to avoid flicker).
            next_state = self._choose_intersection_next(front, left, right)
            reason_before = self.intersection_reason  # keep before clearing
            came_from_back = self.came_from_backwards_corridor

            self.state = next_state

            # Decision taken; clear snapshot to avoid stale data affecting next intersections
            self.intersection_snapshot = None
            self.intersection_reason = None

            self._log_state(self.state)

            # NEW: if turning L/R and this wasn't a front-approach case, roll straight first with yaw locked
            if self.state in (State.TURN_LEFT, State.TURN_RIGHT) and reason_before != 'front_blocked':
                self._pending_turn_state = self.state
                self._preturn_until = now + self.PRETURN_DT
                self._roll_in_forward = (not came_from_back)  # keep direction we arrived with
                self.state = State.PRETURN_STRAIGHT
                self._log_state(self.state)

            if self.state in (State.TURN_LEFT, State.TURN_RIGHT):
                # Set turn target
                sign = +1.0 if self.state == State.TURN_LEFT else -1.0
                self.turn_target_yaw = ang_wrap(self.yaw + sign * self.TURN_ANGLE)
            elif self.state == State.GO_STRAIGHT:
                self._nudge_until = now + self.NUDGE_DT  # brief forward nudge before EXITING_INTERSECTION
            elif self.state == State.GO_BACKWARDS:
                self._nudge_until = now + self.NUDGE_DT  # brief backward nudge before EXITING_INTERSECTION

        elif self.state == State.PRETURN_STRAIGHT:
            # YAW LOCKED: drive straight for PRETURN_DT seconds, keeping the same straight direction we arrived with.
            if now < self._preturn_until:
                vx = self.V_CORRIDOR if getattr(self, '_roll_in_forward', True) else -self.V_CORRIDOR
                self._send_vel(vx, 0.0, 0.0)  # wz = 0 (locked)
            else:
                # Time to perform the queued turn.
                if self._pending_turn_state is None:
                    # Safety fallback
                    self.state = State.GO_STRAIGHT
                    self._nudge_until = now + self.NUDGE_DT
                    self._log_state(self.state)
                else:
                    self.state = self._pending_turn_state
                    self._pending_turn_state = None
                    # Establish the turn target yaw
                    sign = +1.0 if self.state == State.TURN_LEFT else -1.0
                    self.turn_target_yaw = ang_wrap(self.yaw + sign * self.TURN_ANGLE)
                    self._log_state(self.state)

        elif self.state == State.TURN_LEFT or self.state == State.TURN_RIGHT:
            # Perform stationary turn toward target yaw
            if self.turn_target_yaw is None:
                # Safety: set a target if missing
                sign = +1.0 if self.state == State.TURN_LEFT else -1.0
                self.turn_target_yaw = ang_wrap(self.yaw + sign * self.TURN_ANGLE)

            err = ang_wrap(self.turn_target_yaw - self.yaw)
            if abs(err) > math.radians(3.0):
                # Turn in the direction of error at fixed rate
                wz = self.TURN_RATE if err > 0.0 else -self.TURN_RATE
                self._send_vel(0.0, 0.0, wz)
            else:
                # Turn complete: stop yaw, then brief forward nudge
                self._send_vel(0.0, 0.0, 0.0)
                self.turn_target_yaw = None
                self.state = State.GO_STRAIGHT
                self._nudge_until = now + self.NUDGE_DT
                self._log_state(self.state)

        elif self.state == State.GO_STRAIGHT:
            # Brief forward velocity, then EXITING_INTERSECTION
            if now < self._nudge_until:
                self._send_vel(self.V_NUDGE, 0.0, 0.0)
            else:
                self.state = State.EXITING_INTERSECTION
                self._exiting_forward = True
                self._log_state(self.state)

        elif self.state == State.GO_BACKWARDS:
            # Brief backward velocity, then EXITING_INTERSECTION
            if now < self._nudge_until:
                self._send_vel(-self.V_NUDGE, 0.0, 0.0)
            else:
                self.state = State.EXITING_INTERSECTION
                self._exiting_forward = False
                self._log_state(self.state)

        elif self.state == State.EXITING_INTERSECTION:
            # Keep moving straight (no centring) until both side walls are visible (finite)
            if self._exiting_forward:
                self._send_vel(self.V_CORRIDOR, 0.0, 0.0)
            else:
                self._send_vel(-self.V_CORRIDOR, 0.0, 0.0)

            if (left < self.SIDE_OPEN) and (right < self.SIDE_OPEN):
                # Enter corridor mode with centring (forward/back as per last action)
                if self._exiting_forward:
                    self.state = State.STRAIGHT_THROUGH_CORRIDOR
                    self.came_from_backwards_corridor = False
                else:
                    self.state = State.BACKWARDS_THROUGH_CORRIDOR
                    self.came_from_backwards_corridor = True
                self._log_state(self.state)

        else:
            # Fallback safety
            self._send_vel(0.0, 0.0, 0.0)

    # --------------- Control/logic utilities ----------------
    def _corridor_yaw_control(self, left, right, forward=True):
        """
        Centre between side walls using left/right ranges.
        If right > left (more space on the right, you're closer to the left wall),
        you should yaw RIGHT (negative wz). Hence the leading minus sign.
        For backwards motion we flip the sign so the behaviour is mirrored.
        """
        # difference: positive when more space on right than left
        diff = (right - left)

        # small deadband to avoid twitch
        if abs(diff) < 0.02:
            diff = 0.0

        # correct sign: negative turns right when right-left > 0 (closer to left wall)
        wz_cmd = -self.K_YAW * diff

        # flip when reversing so the “keep straight” logic feels the same
        if not forward:
            wz_cmd = -wz_cmd

        # clamp
        if wz_cmd > self.MAX_YAW:
            wz_cmd = self.MAX_YAW
        elif wz_cmd < -self.MAX_YAW:
            wz_cmd = -self.MAX_YAW

        return wz_cmd

    def _choose_intersection_next(self, front, left, right):
        """
        Decide next state using the snapshot captured on INTERSECTION entry.
        Mirrors forward/back logic WITHOUT hysteresis:
        - If trigger was front_blocked -> forbid GO_STRAIGHT.
        - If trigger was back_blocked  -> forbid GO_BACKWARDS.
        Availability (no hysteresis):
          forward_ok  = f_s >= FRONT_T
          backward_ok = b_s >= FRONT_T
          right_ok    = side thresh
          left_ok     = side thresh
        Priorities:
        - Normal: forward > right > left > backwards
        - From backwards corridor: backwards > right > left > forward
        """
        # Use snapshot if available
        if self.intersection_snapshot is not None:
            f_s, l_s, b_s, r_s = self.intersection_snapshot
        else:
            rng = self._ranges()
            if rng is not None:
                f_s, l_s, b_s, r_s = rng
            else:
                return State.GO_BACKWARDS if self.came_from_backwards_corridor else State.GO_STRAIGHT

        # Availability (no hysteresis)
        forward_ok  = (f_s >= self.FRONT_T)
        backward_ok = (b_s >= self.FRONT_T)
        right_ok = (r_s >= self.SIDE_OPEN)
        left_ok  = (l_s >= self.SIDE_OPEN)

        # Forbid the direction that actually triggered the intersection
        if self.intersection_reason == 'front_blocked':
            forward_ok = False
        elif self.intersection_reason == 'back_blocked':
            backward_ok = False

        # NEW priorities:
        if self.came_from_backwards_corridor:
            # Right > Backwards > Left > Forward
            if right_ok:
                return State.TURN_RIGHT
            if backward_ok:
                return State.GO_BACKWARDS
            if left_ok:
                return State.TURN_LEFT
            return State.GO_STRAIGHT
        else:
            # Right > Forward > Left > Backwards
            if right_ok:
                return State.TURN_RIGHT
            if forward_ok:
                return State.GO_STRAIGHT
            if left_ok:
                return State.TURN_LEFT
            return State.GO_BACKWARDS


def main():
    rclpy.init()
    node = CorridorExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._send_vel(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
