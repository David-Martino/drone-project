from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    world_frame_id_arg = DeclareLaunchArgument('world_frame_id', default_value='world')
    cloud_in_arg       = DeclareLaunchArgument('cloud_in',       default_value='/crazyflie_real/pointcloud')

    world_frame_id = LaunchConfiguration('world_frame_id')
    cloud_in       = LaunchConfiguration('cloud_in')

    octomap = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        remappings=[('/cloud_in', cloud_in)],
        parameters=[
            {'frame_id': world_frame_id},
            {'resolution': 0.050},
            {'sensor_model/max_range': 3.5},
            {'publish_free_space': True},
        ],
    )
    return LaunchDescription([world_frame_id_arg, cloud_in_arg, octomap])
