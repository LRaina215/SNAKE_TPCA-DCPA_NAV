import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('linefit_ground_segmentation_ros')
    params_file = os.path.join(bringup_dir, 'launch', 'segmentation_params_real.yaml')

    node_start_cmd = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('cloud_in', '/livox/lidar_no_body'),
            ('input_topic', '/livox/lidar_no_body'),
            ('obstacles', '/segmentation/obstacle'),
            ('ground', '/segmentation/ground')
        ]
    )

    ld = LaunchDescription()
    ld.add_action(node_start_cmd)
    return ld
