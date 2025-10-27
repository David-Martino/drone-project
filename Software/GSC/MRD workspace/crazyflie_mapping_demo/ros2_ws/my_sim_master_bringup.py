# bringup_all.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.conditions import IfCondition


def generate_launch_description():
    # ---- Central args you can still override if needed elsewhere ----
    world_frame_id_arg = DeclareLaunchArgument('world_frame_id', default_value='map')
    body_frame_id_arg  = DeclareLaunchArgument('body_frame_id',  default_value='crazyflie/odom')
    odom_topic_arg     = DeclareLaunchArgument('odom_topic',     default_value='/crazyflie/odom')
    scan_topic_arg     = DeclareLaunchArgument('scan_topic',     default_value='/crazyflie/scan')
    cloud_in_arg       = DeclareLaunchArgument('cloud_in',       default_value='/crazyflie/pointcloud')
    height_offset_arg  = DeclareLaunchArgument('height_offset',  default_value='0.1')

    world_frame_id = LaunchConfiguration('world_frame_id')
    body_frame_id  = LaunchConfiguration('body_frame_id')
    odom_topic     = LaunchConfiguration('odom_topic')
    scan_topic     = LaunchConfiguration('scan_topic')
    cloud_in       = LaunchConfiguration('cloud_in')
    height_offset  = LaunchConfiguration('height_offset')

    # 1) multiranger -> pointcloud (kept parameterised; harmless if unused elsewhere)
    multiranger_pc = Node(
        package='crazyflie_pointcloud',
        executable='multiranger_to_pointcloud',
        name='multiranger_to_pointcloud',
        output='screen',
        parameters=[{
            'scan_topic':         scan_topic,
            'odom_topic':         odom_topic,
            'output_cloud_topic': cloud_in,
            'world_frame_id':     world_frame_id,
            'body_frame_id':      body_frame_id,
        }]
    )

    # 2) Include your edited cf3d_nav_full.launch.py (receives args)
    cf3d_nav_share = get_package_share_directory('cf3d_nav')
    cf3d_nav_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cf3d_nav_share, 'launch', 'cf3d_nav_full.launch.py')
        ),
        launch_arguments={
            'world_frame_id': world_frame_id,
            'cloud_in':       cloud_in,
        }.items()
    )

    # 3) Include simple_mapper_simulation.launch.py exactly as-is (no params)
    bringup_share = get_package_share_directory('crazyflie_ros2_multiranger_bringup')
    simple_mapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'simple_mapper_simulation.launch.py')
        )
    )

    # 4) Standalone Python: gc_vel_mux.py (same style & directory as before)
    vel_mux = ExecuteProcess(
        cmd=[
            'python3', 'gc_vel_mux.py',
            '--ros-args',
            '-p', ['odom_topic:=', odom_topic],
            '-p', ['height_offset:=', height_offset],
        ],
        cwd=['.'],
        output='screen',
        shell=False,
    )

    # 5) Standalone Python: teleop_twist_keyboard_z.py (unchanged)
    teleop = ExecuteProcess(
        prefix='gnome-terminal --',
        cmd=['bash', '-lc', 'python3 teleop_twist_keyboard_z.py'],
        output='screen',
        shell=False,
    )

    return LaunchDescription([
        world_frame_id_arg, body_frame_id_arg, odom_topic_arg, scan_topic_arg,
        cloud_in_arg, height_offset_arg,
        multiranger_pc,
        cf3d_nav_full,
        simple_mapper,
        vel_mux,
        teleop,
    ])
