from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            name='target_frame', default_value='base_link',
            description='Frame to project the point cloud into before creating /scan'
        ),
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in',  ['/segmentation/obstacle']),
                        ('scan',  ['/scan'])],
            # remappings=[('cloud_in',  ['/livox/lidar_no_body']),
            #             ('scan',  ['/scan'])],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'target_frame': LaunchConfiguration('target_frame'),
                'transform_tolerance': 0.2,
                'min_height': 0.05,
                'max_height': 1.0,
                'angle_min': -3.14159,  # -M_PI/2 PHASE1
                'angle_max': 3.14159,  # M_PI/2 
                'angle_increment': 0.0043,  # M_PI/360.0 
                'scan_time': 0.3333,
                'range_min': 0.23,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
