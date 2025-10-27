#!/usr/bin/env python3
"""
level_gimbal_py.py — Keep a Gazebo (Harmonic) link level in roll/pitch using Python + gz-transport.

This script:
 - Resolves two entities by name: follow_link (e.g., 'crazyflie/body') and controlled_link (e.g., 'multiranger_link')
 - Subscribes to /world/<world>/pose/info for world poses
 - Each tick, extracts yaw from the follow_link, and publishes a Pose for the controlled_link to /world/<world>/set_pose
   with roll=0, pitch=0, yaw=follow_yaw, position = current controlled_link position

Requires: gz-transport Python bindings (Gazebo Harmonic).

Usage:
  ./level_gimbal_py.py --world my_world \
    --follow crazyflie/body \
    --control multiranger_link

To run alongside ros2, start it in a separate terminal after the sim starts (or launch it via a process action).
"""
import argparse
import math
import threading
import time

# Gazebo msgs/transport python bindings (package names can vary by distro; try common forms)
try:
    import gz.transport as gz_transport
    import gz.msgs.pose_pb2 as gz_pose_pb2
    import gz.msgs.entity_pb2 as gz_entity_pb2
    import gz.msgs.pose_v_pb2 as gz_pose_v_pb2
except Exception:
    # Fallback to versioned modules (adjust if your distro uses a different major version)
    import gz.transport as gz_transport
    import gz.msgs.pose_pb2 as gz_pose_pb2
    import gz.msgs.entity_pb2 as gz_entity_pb2
    import gz.msgs.pose_v_pb2 as gz_pose_v_pb2


def euler_from_quat(qw, qx, qy, qz):
    # Return roll, pitch, yaw from quaternion
    # Gazebo uses (x, y, z, w) ordering in msgs; be careful.
    # Here we pass in qw,qx,qy,qz explicitly.
    # Formulas from standard conversions.
    # roll (x)
    sinr_cosp = 2.0 * (qw*qx + qy*qz)
    cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y)
    sinp = 2.0 * (qw*qy - qz*qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z)
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class LevelGimbal:
    def __init__(self, world, follow_name, control_name):
        self.world = world
        self.follow_name = follow_name
        self.control_name = control_name
        self.follow_id = None
        self.control_id = None

        self.node = gz_transport.Node()

        self.pose_topic = f"/world/{self.world}/pose/info"
        self.set_pose_topic = f"/world/{self.world}/set_pose"
        self.entity_service = f"/world/{self.world}/entity"  # request: gz.msgs.Entity (name), reply: gz.msgs.Entity (id)

        self._mutex = threading.Lock()
        self._last_control_pose = None
        self._last_follow_yaw = None

        # Subscribe to world pose info (Pose_V message with repeated Pose)
        def _pose_cb(msg):
            # msg is gz.msgs.Pose_V
            # Each element: Pose { name, id, position {x,y,z}, orientation {x,y,z,w} }
            got_follow = False
            got_control = False
            with self._mutex:
                for p in msg.pose:
                    if self.follow_id is not None and p.id == self.follow_id:
                        qw = p.orientation.w; qx = p.orientation.x; qy = p.orientation.y; qz = p.orientation.z
                        _, _, yaw = euler_from_quat(qw, qx, qy, qz)
                        self._last_follow_yaw = yaw
                        got_follow = True
                    if self.control_id is not None and p.id == self.control_id:
                        self._last_control_pose = p
                        got_control = True
            # Optionally, could trigger an immediate publish here, but we do it on a timer loop.

        self.node.Subscribe(self.pose_topic, _pose_cb)

        # Prepare publisher for set_pose
        self.node.Advertise(self.set_pose_topic, gz_pose_pb2.Pose)

    def _lookup_entity_id(self, name):
        # Call the /world/<world>/entity service with a gz.msgs.Entity containing only name
        req = gz_entity_pb2.Entity()
        req.name = name
        rep = gz_entity_pb2.Entity()
        result = self.node.Request(self.entity_service, req, 1000, rep)  # 1000 ms timeout
        if not result:
            return None
        if rep.id == 0:
            return None
        return rep.id

    def resolve_ids(self):
        # Try a few times at startup in case sim isn't fully ready
        for _ in range(50):
            self.follow_id = self._lookup_entity_id(self.follow_name)
            self.control_id = self._lookup_entity_id(self.control_name)
            if self.follow_id and self.control_id:
                print(f"[level_gimbal_py] Resolved follow='{self.follow_name}' -> {self.follow_id}, "
                      f"control='{self.control_name}' -> {self.control_id}")
                return True
            time.sleep(0.2)
        print("[level_gimbal_py] WARNING: Could not resolve entity IDs. Check names/world.")
        return False

    def spin(self, rate_hz=100.0):
        dt = 1.0 / rate_hz
        while True:
            yaw = None
            control_pose = None
            with self._mutex:
                if self._last_follow_yaw is not None:
                    yaw = self._last_follow_yaw
                if self._last_control_pose is not None:
                    control_pose = self._last_control_pose

            if yaw is not None and control_pose is not None:
                # Build a Pose command for the controlled entity: keep its position, set orientation to yaw-only
                cmd = gz_pose_pb2.Pose()
                cmd.name = self.control_name
                cmd.id = self.control_id
                cmd.position.x = control_pose.position.x
                cmd.position.y = control_pose.position.y
                cmd.position.z = control_pose.position.z

                # Convert yaw-only to quaternion
                half = 0.5 * yaw
                cmd.orientation.x = 0.0
                cmd.orientation.y = 0.0
                cmd.orientation.z = math.sin(half)
                cmd.orientation.w = math.cos(half)

                self.node.Publish(self.set_pose_topic, cmd)

            time.sleep(dt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True, help="Gazebo world name (as seen in /world/<world>/...)")
    ap.add_argument("--follow", required=True, help="Entity name to follow for yaw (e.g., 'crazyflie/body')")
    ap.add_argument("--control", required=True, help="Entity name to keep level (e.g., 'multiranger_link')")
    ap.add_argument("--rate", type=float, default=100.0, help="Control loop rate (Hz)")
    args = ap.parse_args()

    lg = LevelGimbal(args.world, args.follow, args.control)
    lg.resolve_ids()
    lg.spin(rate_hz=args.rate)


if __name__ == "__main__":
    main()
