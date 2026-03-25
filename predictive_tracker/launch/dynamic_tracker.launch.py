from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true.'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('predictive_tracker'),
                'config',
                'dynamic_tracker.yaml'
            ]),
            description='Full path to the predictive tracker parameter file.'
        ),
        Node(
            package='predictive_tracker',
            executable='dynamic_tracker_node',
            name='predictive_tracker',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        )
    ])
