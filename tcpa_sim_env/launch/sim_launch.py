import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('tcpa_sim_env')
    gazebo_share = get_package_share_directory('gazebo_ros')

    default_world = os.path.join(package_share, 'worlds', 'dynamic_test.world')
    robot_xacro = os.path.join(package_share, 'urdf', 'robot.xacro')
    gzserver_launch = os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')
    gzclient_launch = os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')

    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    lidar_backend = LaunchConfiguration('lidar_backend')
    scene_mode = LaunchConfiguration('scene_mode')
    use_livox_plugin = IfCondition(PythonExpression(["'", lidar_backend, "' == 'livox_plugin'"]))

    robot_description = Command([
        'xacro ',
        robot_xacro,
        ' lidar_backend:=',
        lidar_backend,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to false to run Gazebo headless.'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Full path to the Gazebo world file.'
        ),
        DeclareLaunchArgument(
            'lidar_backend',
            default_value='livox_plugin',
            description='LiDAR backend to use: livox_plugin or gpu_ray.'
        ),
        DeclareLaunchArgument(
            'scene_mode',
            default_value='dynamic_test',
            description='Obstacle motion scene profile: dynamic_test, narrow_corridor, or random_crowd.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzserver_launch),
            launch_arguments={
                'world': world,
                'verbose': 'false',
                'pause': 'false',
                'factory': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzclient_launch),
            condition=IfCondition(gui),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),
        Node(
            package='tcpa_sim_env',
            executable='obstacle_mover.py',
            name='obstacle_mover',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'scene_mode': scene_mode,
            }],
        ),
        Node(
            package='tcpa_sim_env',
            executable='pointcloud_relay.py',
            name='livox_pointcloud_relay',
            output='screen',
            condition=use_livox_plugin,
            parameters=[{
                'input_topic': '/livox/lidar_custom_PointCloud2',
                'output_topic': '/livox/lidar',
                'use_sim_time': True,
            }],
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_tcpa_robot',
            output='screen',
            arguments=[
                '-entity', 'tcpa_robot',
                '-topic', 'robot_description',
                '-x', '-4.0',
                '-y', '0.0',
                '-z', '0.1',
            ],
        ),
    ])
