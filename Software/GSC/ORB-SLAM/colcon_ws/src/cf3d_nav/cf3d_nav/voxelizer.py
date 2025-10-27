#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class VoxelGridBuilder(Node):
    """
    Subscribes to OctoMap's /occupied_cells_vis_array (MarkerArray of cubes)
    and builds a 3D occupancy grid in 'map' frame. Publishes a light-weight
    grid descriptor and exposes a service-like topic for the planner.
    """
    def __init__(self):
        super().__init__('voxel_grid_builder')
        self.declare_parameter('resolution', 0.010)     # meters (must match octomap resolution)
        self.declare_parameter('inflate_radius', 0.03) # inflate obstacles by drone radius
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('grid_pub', '/cf3d/grid_vis')

        self.res = float(self.get_parameter('resolution').value)
        self.inflate = float(self.get_parameter('inflate_radius').value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.grid_pub_topic = self.get_parameter('grid_pub').get_parameter_value().string_value

        self.sub = self.create_subscription(MarkerArray, '/occupied_cells_vis_array_own', self.on_markers, 10)
        self.grid_vis_pub = self.create_publisher(MarkerArray, self.grid_pub_topic, 1)

        # internal grid
        self.occ = None  # 3D boolean numpy array
        self.origin = None  # (x0,y0,z0) world origin of grid
        self.size = None    # (nx,ny,nz)

    def on_markers(self, marr: MarkerArray):
        pts = []
        for m in marr.markers:
            # each marker contains many points in 'points' or uses pose+scale
            # occupied_cells_vis_array typically sets points with marker type CUBE_LIST
            if m.type == Marker.CUBE_LIST:
                for p in m.points:
                    pts.append((p.x, p.y, p.z))
        if not pts:
            return

        P = np.array(pts, dtype=np.float32)

        # Compute AABB and grid
        min_xyz = P.min(axis=0) - self.inflate - self.res
        max_xyz = P.max(axis=0) + self.inflate + self.res
        size_xyz = max_xyz - min_xyz
        nx, ny, nz = np.ceil(size_xyz / self.res).astype(int) + 1

        # allocate grid
        self.occ = np.zeros((nx, ny, nz), dtype=np.uint8)
        self.origin = min_xyz
        self.size = (nx, ny, nz)

        # mark occupied cells and inflate
        def w2g(xyz):
            ijk = np.floor((xyz - self.origin) / self.res).astype(int)
            return ijk

        # base occupancy
        base_ids = w2g(P)
        base_ids = np.unique(base_ids, axis=0)

        # inflation radius in voxels
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

        # publish a sparse viz of inflated occupied grid (downsample for RViz)
        self.publish_grid_viz(skip=max(1, int(0.15 / self.res)))  # show ~15cm stride

    def publish_grid_viz(self, skip=1):
        if self.occ is None:
            return
        nx, ny, nz = self.size
        pts = []
        for i in range(0, nx, skip):
            xs = self.origin[0] + (i + 0.5) * self.res
            occ_i = self.occ[i]            # shape (ny, nz)
            jj, kk = np.where(occ_i == 1)  # <- was 3 outputs; it's 2D so only 2!
            for j, k in zip(jj, kk):
                y = self.origin[1] + (j + 0.5) * self.res
                z = self.origin[2] + (k + 0.5) * self.res
                pts.append((xs, y, z))

        marr = MarkerArray()
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'cf3d_grid'
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.scale.x = self.res
        m.scale.y = self.res
        m.scale.z = self.res
        m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 0.5
        for x, y, z in pts:
            p = Point(); p.x = float(x); p.y = float(y); p.z = float(z)
            m.points.append(p)
        marr.markers.append(m)
        self.grid_vis_pub.publish(marr)


def main():
    rclpy.init()
    n = VoxelGridBuilder()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
