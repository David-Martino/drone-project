from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    octo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cf3d_nav'), 'launch', 'octomap_mapping.launch.py')
        )
    )
    voxelizer = Node(package='cf3d_nav', executable='cf3d_voxelizer', name='cf3d_voxelizer',
                     parameters=[{'resolution': 0.001, 'inflate_radius': 0.02, 'frame_id':'map'}])

    return LaunchDescription([octo, voxelizer, ])
