from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true.')

    # Declare the RViz argument
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to launch RViz.')

    enable_lidar_filter_arg = DeclareLaunchArgument(
        'enable_lidar_filter', default_value='false',
        description='Enable /livox/lidar -> /livox/lidar_no_body filtering for the linefit pipeline.')

    # This filter is only needed by the legacy linefit segmentation path.
    filter_node = Node(
        package='rm_lidar_filter',
        executable='lidar_filter',
        name='lidar_filter_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('enable_lidar_filter'))
    )

    # Node parameters, including those from the YAML configuration file
    laser_mapping_params = [
        PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'config', 'mid360.yaml'
        ]),
        {
            'use_imu_as_input': False,  # Change to True to use IMU as input of Point-LIO
            # 仿真专用：强制 laserMapping 跟随 Gazebo /clock。
            'use_sim_time': True,
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 5,  # 仿真降载：减少参与建图的点，避免 IMU 队列堆积
            'space_down_sample': True,
            'filter_size_surf': 0.5,  # 仿真降载：维持较大的体素滤波
            'filter_size_map': 0.5,  # 仿真降载：维持较大的地图体素滤波
            'cube_side_length': 1000.0,  # Option: 1000
            'runtime_pos_log_enable': False,  # Option: True
        }
    ]

    # Node definition for laserMapping with Point-LIO
    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
        remappings=[
            ('odom', '/odom_livox'),
        ],
        # prefix='gdb -ex run --args'
    )

    # Conditional RViz node launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )

    # Assemble the launch description
    ld = LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        enable_lidar_filter_arg,
        filter_node,
        laser_mapping_node,
        # GroupAction(
        #    actions=[rviz_node],
        #    condition=IfCondition(LaunchConfiguration('rviz'))
        # ),
    ])

    return ld
