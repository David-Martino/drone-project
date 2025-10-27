#!/usr/bin/env python3
import math, random, numpy as np
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from tf_transformations import euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 (keeps tf2 buffer happy)


def bresenham3d(p0i, p1i):
    """Integer 3D Bresenham line from p0i (i,j,k) to p1i (i,j,k)."""
    (x0, y0, z0) = map(int, p0i)
    (x1, y1, z1) = map(int, p1i)
    dx = abs(x1 - x0); dy = abs(y1 - y0); dz = abs(z1 - z0)
    sx = 1 if x1 >= x0 else -1
    sy = 1 if y1 >= y0 else -1
    sz = 1 if z1 >= z0 else -1

    if dx >= dy and dx >= dz:
        p1 = 2 * dy - dx
        p2 = 2 * dz - dx
        while x0 != x1:
            x0 += sx
            if p1 >= 0: y0 += sy; p1 -= 2 * dx
            if p2 >= 0: z0 += sz; p2 -= 2 * dx
            p1 += 2 * dy; p2 += 2 * dz
            yield (x0, y0, z0)
    elif dy >= dx and dy >= dz:
        p1 = 2 * dx - dy
        p2 = 2 * dz - dy
        while y0 != y1:
            y0 += sy
            if p1 >= 0: x0 += sx; p1 -= 2 * dy
            if p2 >= 0: z0 += sz; p2 -= 2 * dy
            p1 += 2 * dx; p2 += 2 * dz
            yield (x0, y0, z0)
    else:
        p1 = 2 * dy - dz
        p2 = 2 * dx - dz
        while z0 != z1:
            z0 += sz
            if p1 >= 0: y0 += sy; p1 -= 2 * dz
            if p2 >= 0: x0 += sx; p2 -= 2 * dz
            p1 += 2 * dy; p2 += 2 * dx
            yield (x0, y0, z0)


class CF3DExplorer(Node):
    """
    Minimal 3D explorer:
      - Rebuilds a local voxel occupancy from /occupied_cells_vis_array
      - Chooses short-range random 3D goals that pass a straight-line free check
      - Applies simple repulsive 'nudge' from nearby occupied voxels
      - Publishes /crazyflie/cmd_vel Twist until goal reached, then resamples
    """

    def __init__(self):
        super().__init__('cf3d_explorer3d')

        # --- Params you’ll likely tune ---
        self.declare_parameter('cmd_vel_topic', '/crazyflie/cmd_vel')
        self.declare_parameter('odom_topic', '/crazyflie/odom')
        self.declare_parameter('markers_topic', '/occupied_cells_vis_array')
        self.declare_parameter('frame_id', 'map')   # marker frame
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('resolution', 0.05)  # must match your octomap resolution
        self.declare_parameter('inflate_radius', 0.0)  # must match voxelizer inflate
        self.declare_parameter('goal_radius', 0.25)  # when within this, resample goal
        self.declare_parameter('cruise_speed', 0.5)  # m/s cap
        self.declare_parameter('kp', 0.8)            # proportional gain to goal
        self.declare_parameter('sample_radius_xy', 3.0)  # local goal sampling box
        self.declare_parameter('sample_radius_z', 1.5)
        self.declare_parameter('z_min', 0.10)        # keep above floor
        self.declare_parameter('z_max', 2.50)        # keep below "ceiling"
        self.declare_parameter('repulse_radius', 0.05)   # m (within this, push away)
        self.declare_parameter('repulse_gain', 0.3)      # weight of repulsion
        self.declare_parameter('loop_rate_hz', 20.0)



        # --- Read params ---
        self.res = float(self.get_parameter('resolution').value)
        self.inflate = float(self.get_parameter('inflate_radius').value)
        self.goal_r = float(self.get_parameter('goal_radius').value)
        self.vmax = float(self.get_parameter('cruise_speed').value)
        self.kp = float(self.get_parameter('kp').value)
        self.sr_xy = float(self.get_parameter('sample_radius_xy').value)
        self.sr_z = float(self.get_parameter('sample_radius_z').value)
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self.rep_r = float(self.get_parameter('repulse_radius').value)
        self.rep_k = float(self.get_parameter('repulse_gain').value)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        markers_topic = self.get_parameter('markers_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        # --- IO ---
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.sub_markers = self.create_subscription(MarkerArray, markers_topic, self.on_markers, 10)
        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.on_odom, 10)

        # --- TF buffer (optional map->odom) ---
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Grid state ---
        self.occ = None            # np.ndarray [nx,ny,nz], uint8
        self.origin = None         # np.array([x0,y0,z0])
        self.size = None           # (nx,ny,nz)
        self.last_grid_stamp = self.get_clock().now()

        # --- Robot state / goal ---
        self.p_map = np.array([0.0, 0.0, 0.3], dtype=np.float32)  # position in map frame
        self.has_odom = False
        self.goal: Optional[np.ndarray] = None

        # Main control loop
        hz = float(self.get_parameter('loop_rate_hz').value)
        self.timer = self.create_timer(1.0 / hz, self.control_step)

        self.get_logger().info('CF3D Explorer initialised.')   

        # --- Stagnation / exploration helpers ---
        self.hop_every = 5.0  # seconds
        self.last_hop = self.get_clock().now()     

        # Optional: long-hop feature 
        self.last_p = self.p_map.copy() #seconds between long attempts
        self.last_progress_check = self.get_clock().now()
        self.stuck_for = 0.0

    # --------- Subscriptions ----------
    def on_markers(self, marr: MarkerArray):
        # Build (or rebuild) occupancy grid from CUBE_LIST markers
        pts = []
        res_from_markers = None
        for m in marr.markers:
            if m.type == Marker.CUBE_LIST and m.points:
                res_from_markers = m.scale.x if m.scale.x > 1e-6 else None
                for p in m.points:
                    pts.append((p.x, p.y, p.z))

        if not pts:
            return

        P = np.array(pts, dtype=np.float32)
        # Allow following Octomap’s true resolution if present
        res = res_from_markers if res_from_markers else self.res

        inflate = max(self.inflate, 0.0)
        min_xyz = P.min(axis=0) - inflate - res
        max_xyz = P.max(axis=0) + inflate + res
        size_xyz = max_xyz - min_xyz
        nx, ny, nz = np.ceil(size_xyz / res).astype(int) + 1

        occ = np.zeros((nx, ny, nz), dtype=np.uint8)
        origin = min_xyz

        def w2g(xyz):
            return np.floor((xyz - origin) / res).astype(int)

        base_ids = np.unique(w2g(P), axis=0)

        # Inflate obstacles by radius (in voxels)
        r = max(0, int(math.ceil(inflate / res)))
        if r > 0:
            off = np.array([(i, j, k)
                            for i in range(-r, r + 1)
                            for j in range(-r, r + 1)
                            for k in range(-r, r + 1)
                            if (i*i + j*j + k*k) <= (r*r + 1)], dtype=int)
            for i, j, k in base_ids:
                ii = i + off[:, 0]; jj = j + off[:, 1]; kk = k + off[:, 2]
                mask = (ii >= 0) & (jj >= 0) & (kk >= 0) & (ii < nx) & (jj < ny) & (kk < nz)
                occ[ii[mask], jj[mask], kk[mask]] = 1
        else:
            occ[base_ids[:, 0], base_ids[:, 1], base_ids[:, 2]] = 1

        # Commit
        self.occ = occ
        self.origin = origin
        self.size = (nx, ny, nz)
        self.res = res  # keep in sync if discovered
        self.last_grid_stamp = self.get_clock().now()

    def on_odom(self, od: Odometry):
        # Transform odom pose into map frame if needed
        px = od.pose.pose.position.x
        py = od.pose.pose.position.y
        pz = od.pose.pose.position.z

        q = od.pose.pose.orientation
        _ = euler_from_quaternion([q.x, q.y, q.z, q.w])  # yaw unused, but handy if you add heading

        # Is odom frame different to map? If yes, try transform.
        if self.odom_frame != self.frame_id:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.frame_id, self.odom_frame, rclpy.time.Time())
                tx = tf.transform.translation
                rx = tf.transform.rotation
                # Only apply translation (ignore rotation for simplicity in this minimal example)
                px = px + tx.x
                py = py + tx.y
                pz = pz + tx.z
            except Exception:
                pass  # if TF missing, we’ll assume frames roughly aligned

        self.p_map = np.array([px, py, pz], dtype=np.float32)
        self.has_odom = True

        
    def occ_density_score(self, g, radius=0.6):
        if self.occ is None: return 0.0
        gi = self.world_to_idx(g)
        if gi is None: return -1e9
        r = max(1, int(radius / self.res))
        nx, ny, nz = self.size
        ix, iy, iz = gi
        i0 = max(0, ix - r); i1 = min(nx - 1, ix + r)
        j0 = max(0, iy - r); j1 = min(ny - 1, iy + r)
        k0 = max(0, iz - r); k1 = min(nz - 1, iz + r)
        # fewer occupied voxels nearby → higher score
        occ_count = int(self.occ[i0:i1+1, j0:j1+1, k0:k1+1].sum())
        return -occ_count  # negative so fewer → larger

    # --------- Helpers ----------
    def world_to_idx(self, xyz: np.ndarray) -> Optional[Tuple[int, int, int]]:
        if self.occ is None:
            return None
        ijk = np.floor((xyz - self.origin) / self.res).astype(int)
        nx, ny, nz = self.size
        if (ijk[0] < 0 or ijk[1] < 0 or ijk[2] < 0 or
            ijk[0] >= nx or ijk[1] >= ny or ijk[2] >= nz):
            return None
        return int(ijk[0]), int(ijk[1]), int(ijk[2])

    def is_free_line(self, a_w: np.ndarray, b_w: np.ndarray) -> bool:
        ai = self.world_to_idx(a_w); bi = self.world_to_idx(b_w)
        if ai is None or bi is None:
            return False
        for i, j, k in bresenham3d(ai, bi):
            if self.occ[i, j, k] == 1:
                return False
        return True

    def sample_goal_near(self) -> Optional[np.ndarray]:
        if self.occ is None:
            return self.p_map + np.array([
                random.uniform(-self.sr_xy, self.sr_xy),
                random.uniform(-self.sr_xy, self.sr_xy),
                np.clip(self.p_map[2] + random.uniform(-self.sr_z, self.sr_z), self.z_min, self.z_max)
            ], dtype=np.float32)

        candidates = []
        for _ in range(40):  # was ~15; try 40 to be choosier
            dx = random.uniform(-self.sr_xy, self.sr_xy)
            dy = random.uniform(-self.sr_xy, self.sr_xy)
            dz = random.uniform(-self.sr_z, self.sr_z)
            g = self.p_map + np.array([dx, dy, dz], dtype=np.float32)
            g[2] = float(np.clip(g[2], self.z_min, self.z_max))
            if self.world_to_idx(g) is not None and self.is_free_line(self.p_map, g):
                candidates.append(g)

        if not candidates:
            return None

        scores = []
        for g in candidates:
            # combine "go far" and "go sparse" (tune weights as you like)
            far = np.linalg.norm(g - self.p_map)
            sparse = self.occ_density_score(g, radius=0.8)
            scores.append(1.0 * far + 0.5 * sparse)
        return candidates[int(np.argmax(scores))]
        # pick the farthest valid goal to push outward
        dists = [np.linalg.norm(g - self.p_map) for g in candidates]
        return candidates[int(np.argmax(dists))]


    def repulsion(self) -> np.ndarray:
        """Inverse-distance repulsion from occupied voxels within repulse_radius."""
        if self.occ is None:
            return np.zeros(3, dtype=np.float32)
        idx = self.world_to_idx(self.p_map)
        if idx is None:
            return np.zeros(3, dtype=np.float32)

        r_vox = max(1, int(math.ceil(self.rep_r / self.res)))
        nx, ny, nz = self.size
        ix, iy, iz = idx
        acc = np.zeros(3, dtype=np.float32)

        i0 = max(0, ix - r_vox); i1 = min(nx - 1, ix + r_vox)
        j0 = max(0, iy - r_vox); j1 = min(ny - 1, iy + r_vox)
        k0 = max(0, iz - r_vox); k1 = min(nz - 1, iz + r_vox)

        for i in range(i0, i1 + 1):
            x = self.origin[0] + (i + 0.5) * self.res
            for j in range(j0, j1 + 1):
                y = self.origin[1] + (j + 0.5) * self.res
                for k in range(k0, k1 + 1):
                    if self.occ[i, j, k] == 0:
                        continue
                    z = self.origin[2] + (k + 0.5) * self.res
                    d = np.array([self.p_map[0] - x, self.p_map[1] - y, self.p_map[2] - z], dtype=np.float32)
                    dist = np.linalg.norm(d) + 1e-6
                    if dist <= self.rep_r:
                        acc += (d / (dist * dist))  # 1/r^2 push
        return acc

    # --------- Control loop ----------
    def control_step(self):

        now = self.get_clock().now()
        dt = (now - self.last_progress_check).nanoseconds / 1e9
        if dt > 2.0:
            moved = float(np.linalg.norm(self.p_map - self.last_p))
            if moved < 0.20:
                self.stuck_for += dt
                # temporarily widen search radii
                self.sr_xy = min(self.sr_xy * 1.35, 6.0)
                self.sr_z  = min(self.sr_z  * 1.35, 3.0)
            else:
                # shrink back toward defaults
                self.sr_xy = max(self.sr_xy * 0.9, 1.5)
                self.sr_z  = max(self.sr_z  * 0.9, 0.8)
                self.stuck_for = 0.0
            self.last_p = self.p_map.copy()
            self.last_progress_check = now

        # Hover if no odom yet
        if not self.has_odom:
            self.cmd_pub.publish(Twist())
            return

        # Refresh / resample goal if needed
        if self.goal is None or self.occ is None:
            self.goal = self.sample_goal_near()

        if self.goal is None:
            # No valid goal found → gently ascend a little to escape
            targ = self.p_map + np.array([0.0, 0.0, 0.3], dtype=np.float32)
        else:
            targ = self.goal

        # Recheck straight line; if blocked, resample
        if self.occ is not None and not self.is_free_line(self.p_map, targ):
            self.goal = self.sample_goal_near()
            if self.goal is not None:
                targ = self.goal

        # Goal reached?
        if np.linalg.norm(targ - self.p_map) < self.goal_r:
            self.goal = self.sample_goal_near()

        # Compute command
        cmd = Twist()

        # Attractive velocity to goal
        if self.goal is not None:
            v_des = (self.goal - self.p_map) * self.kp
        else:
            v_des = np.zeros(3, dtype=np.float32)

        # Add repulsion from nearby voxels
        v_rep = self.repulsion() * self.rep_k
        v = v_des + v_rep

        # Limit Z to stay within bounds
        if self.p_map[2] < self.z_min + 0.05:
            v[2] += 0.3
        if self.p_map[2] > self.z_max - 0.05:
            v[2] -= 0.3

        # Clip speed
        speed = np.linalg.norm(v)
        if speed > 1e-3:
            v = v * (min(self.vmax, speed) / speed)

        cmd.linear.x = float(v[0])
        cmd.linear.y = float(v[1])
        cmd.linear.z = float(v[2])
        # (leave yaw untouched; Crazyflie usually ignores angular if you fly in vel mode)

        self.cmd_pub.publish(cmd)



def main():
    rclpy.init()
    n = CF3DExplorer()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
 