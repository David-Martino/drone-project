# cf3d_nav_full.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # -------- Launch-time arguments (override from parent/master) --------
    world_frame_id_arg = DeclareLaunchArgument('world_frame_id', default_value='map')
    cloud_in_arg       = DeclareLaunchArgument('cloud_in',       default_value='/crazyflie/pointcloud')


    world_frame_id = LaunchConfiguration('world_frame_id')
    cloud_in       = LaunchConfiguration('cloud_in')


    # -------- Include OctoMap (expects its launch to declare world_frame_id + cloud_in) --------
    octomap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cf3d_nav'), 'launch', 'octomap_mapping.launch.py')
        ),
        launch_arguments={
            'world_frame_id': world_frame_id,
            'cloud_in':       cloud_in,
        }.items()
    )

    return LaunchDescription([
        world_frame_id_arg, cloud_in_arg,
        octomap_launch,
    ])
