#!/usr/bin/env python3
import math
import heapq
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

class Planner3D(Node):
    def __init__(self):
        super().__init__('cf3d_planner')
        # Frames & grid
        self.declare_parameter('frame_id', 'crazyflie/odom')   # match your world_frame_id
        self.declare_parameter('resolution', 0.10)              # MUST match octomap_server resolution
        self.declare_parameter('inflate_radius', 0.20)          # drone radius inflation (m)

        # Planning
        self.declare_parameter('step', 0.10)                    # = resolution
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.res      = float(self.get_parameter('resolution').value)
        self.inflate  = float(self.get_parameter('inflate_radius').value)
        self.step     = float(self.get_parameter('step').value)

        # Inputs
        self.sub_start = self.create_subscription(PoseStamped, '/cf3d/start', self.on_start, 10)
        self.sub_goal  = self.create_subscription(PoseStamped, '/cf3d/goal',  self.on_goal,  10)
        # Ingest occupied voxels from OctoMap visualization:
        self.sub_occ   = self.create_subscription(MarkerArray, '/occupied_cells_vis_array', self.on_markers, 10)

        # Outputs
        self.path_pub  = self.create_publisher(Path, '/cf3d/path', 1)
        self.grid_vis_pub = self.create_publisher(MarkerArray, '/cf3d/grid_vis', 1)
        self.way_pub   = self.create_publisher(Marker, '/cf3d/waypoints', 1)

        # State
        self.start = None
        self.goal  = None
        self.occ   = None           # np.uint8 [nx,ny,nz]
        self.origin = None          # np.float32 [3]
        self.size   = None          # (nx,ny,nz)

        self.timer = self.create_timer(0.5, self.try_plan)

    # ===== OctoMap ingestion -> build inflated occupancy grid =====
    def on_markers(self, marr: MarkerArray):
        pts = []
        for m in marr.markers:
            if m.type == Marker.CUBE_LIST:
                for p in m.points:
                    pts.append((p.x, p.y, p.z))
        if not pts:
            return

        P = np.array(pts, dtype=np.float32)
        min_xyz = P.min(axis=0) - self.inflate - self.res
        max_xyz = P.max(axis=0) + self.inflate + self.res
        size_xyz = max_xyz - min_xyz
        nx, ny, nz = np.ceil(size_xyz / self.res).astype(int) + 1

        self.occ = np.zeros((nx, ny, nz), dtype=np.uint8)
        self.origin = min_xyz.astype(np.float32)
        self.size = (nx, ny, nz)

        def w2g(xyz):
            return np.floor((xyz - self.origin) / self.res).astype(int)

        base_ids = w2g(P)
        base_ids = np.unique(base_ids, axis=0)

        r = max(1, int(math.ceil(self.inflate / self.res)))
        off = np.array([(i, j, k)
                        for i in range(-r, r+1)
                        for j in range(-r, r+1)
                        for k in range(-r, r+1)
                        if (i*i + j*j + k*k) <= (r*r+1)], dtype=int)

        for i,j,k in base_ids:
            ii = i + off[:,0]
            jj = j + off[:,1]
            kk = k + off[:,2]
            mask = (ii>=0)&(jj>=0)&(kk>=0)&(ii<nx)&(jj<ny)&(kk<nz)
            self.occ[ii[mask], jj[mask], kk[mask]] = 1

        self.publish_grid_viz()

    def publish_grid_viz(self, skip_vox=1):
        if self.occ is None:
            return
        nx, ny, nz = self.size
        pts = []
        xs = self.origin[0] + (np.arange(0, nx, skip_vox) + 0.5)*self.res
        for idx_i, i in enumerate(range(0, nx, skip_vox)):
            occ_i = self.occ[i]              # (ny, nz)
            jj, kk = np.where(occ_i == 1)
            if jj.size == 0:
                continue
            x = float(xs[idx_i])
            for j,k in zip(jj, kk):
                y = float(self.origin[1] + (j + 0.5) * self.res)
                z = float(self.origin[2] + (k + 0.5) * self.res)
                pts.append((x,y,z))

        marr = MarkerArray()
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'cf3d_grid'
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.scale.x = self.res; m.scale.y = self.res; m.scale.z = self.res
        m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 0.5
        for x,y,z in pts:
            pt = Point(); pt.x = x; pt.y = y; pt.z = z
            m.points.append(pt)
        marr.markers.append(m)
        self.grid_vis_pub.publish(marr)

    # ===== Start/Goal =====
    def on_start(self, msg: PoseStamped):
        self.start = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float32)

    def on_goal(self, msg: PoseStamped):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float32)

    # ===== Planner tick =====
    def try_plan(self):
        if self.occ is None or self.origin is None:
            self.get_logger().warn("No grid yet.")
            return
        if self.start is None or self.goal is None:
            return

        def w2g(p):
            return np.floor((p - self.origin) / self.step).astype(int)

        def g2w(i,j,k):
            return self.origin + self.step * (np.array([i+0.5, j+0.5, k+0.5], dtype=np.float32))

        s = w2g(self.start)
        g = w2g(self.goal)
        nx, ny, nz = self.size
        for v in (s, g):
            if (v[0]<0 or v[1]<0 or v[2]<0 or v[0]>=nx or v[1]>=ny or v[2]>=nz):
                self.get_logger().error("Start or goal outside grid.")
                return
        if self.occ[s[0], s[1], s[2]] == 1 or self.occ[g[0], g[1], g[2]] == 1:
            self.get_logger().error("Start/goal in collision.")
            return

        # 26-neighbor A*
        neigh = [(i,j,k) for i in (-1,0,1) for j in (-1,0,1) for k in (-1,0,1) if not (i==0 and j==0 and k==0)]
        def h(a,b):
            d = a-b
            return float(np.sqrt((d**2).sum()))

        openq = []
        s_key = tuple(s.tolist())
        g_key = tuple(g.tolist())
        heapq.heappush(openq, (0.0, s_key))
        gscore = {s_key: 0.0}
        came = {s_key: None}

        found = None
        iters = 0
        while openq and iters < 500000:
            _, cur = heapq.heappop(openq)
            iters += 1
            if cur == g_key:
                found = cur
                break
            ci,cj,ck = cur
            for di,dj,dk in neigh:
                ni,nj,nk = ci+di, cj+dj, ck+dk
                if ni<0 or nj<0 or nk<0 or ni>=nx or nj>=ny or nk>=nz:
                    continue
                if self.occ[ni,nj,nk] == 1:
                    continue
                step_cost = math.sqrt(di*di + dj*dj + dk*dk)
                nkey = (ni,nj,nk)
                cand_g = gscore[cur] + step_cost
                if cand_g < gscore.get(nkey, 1e18):
                    gscore[nkey] = cand_g
                    came[nkey] = cur
                    f = cand_g + h(np.array([ni,nj,nk], dtype=np.float32), np.array(g, dtype=np.float32))
                    heapq.heappush(openq, (f, nkey))

        if found is None:
            self.get_logger().error("No path found.")
            return

        # reconstruct
        voxels = []
        v = found
        while v is not None:
            voxels.append(v)
            v = came[v]
        voxels.reverse()

        # (optional) quick smoothing omitted for clarity

        pts = [g2w(i,j,k) for (i,j,k) in voxels]

        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for x,y,z in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(z)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

        m = Marker()
        m.header = path.header
        m.ns = "cf3d_waypoints"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.scale.x = self.step * 0.8
        m.scale.y = self.step * 0.8
        m.scale.z = self.step * 0.8
        m.color.r = 0.2; m.color.g = 1.0; m.color.b = 0.2; m.color.a = 0.9
        for x, y, z in pts:
            p = Point(); p.x=float(x); p.y=float(y); p.z=float(z)
            m.points.append(p)
        self.way_pub.publish(m)

def main():
    rclpy.init()
    node = Planner3D()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
