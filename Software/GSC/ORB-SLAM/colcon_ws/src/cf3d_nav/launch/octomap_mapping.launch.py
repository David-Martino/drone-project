from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    filter_node = Node(
        package='cf3d_cloud_filters',
        executable='temporal_pc_filter',
        name='temporal_pc_filter',
        parameters=[
            {'input_topic': '/slam/pointcloud'},
            {'output_topic': '/slam/pointcloud_filtered'},
            {'leaf_size': 0.03},
            {'sor_mean_k': 18},
            {'sor_stddev': 1.2},
            {'ror_radius': 0.08},
            {'ror_min_neighbors': 3},
            {'use_temporal_window': True},
            {'window_size': 5},
        ]
    )

    octo = Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            remappings=[
                ('cloud_in', '/slam/pointcloud_filtered'),
                ('occupied_cells_vis_array', '/occupied_cells_vis_array_own'),
            ],
            parameters=[
                # Make voxels coarser to hide single-point noise (try 0.05–0.20)
                {'resolution': 0.0005},
                {'frame_id': 'odom'},

                # 1) Hard gates on what gets inserted
                {'sensor_model/max_range': 6.0},        # drop far, noisy hits
                {'pointcloud_min_z': -0.2},             # keep only Z in [min,max]
                {'pointcloud_max_z':  2.5},

                # 2) Be stricter about declaring OCCUPIED
                # Smaller 'hit' increment => needs more confirmations before a voxel flips to occupied
                {'sensor_model/hit': 0.40},             # default is ~0.7
                {'sensor_model/miss': 0.60},            # keep freespace clearing reasonable
                {'sensor_model/min': 0.12},             # clamp lower bound (default)
                {'sensor_model/max': 0.97},             # clamp upper bound (default)

                # Optional: publish free voxels too (for debugging)
                # {'publish_free_space': True},
                        # === Visualization colours ===
                {'color/rainbow': True},                   # enable rainbow colouring by height
                {'color/height_map': True},                # use height map colour scheme
                {'color/z_min': -0.05},                     # lowest colour level (m)
                {'color/z_max': 0.3},                      # highest colour level (m)
                {'publish_free_space': False},             # optional (for clarity)
            ]
        )

    return LaunchDescription([filter_node, octo])
