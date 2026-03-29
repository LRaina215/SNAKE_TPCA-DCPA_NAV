import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('navi')
    launch_dir = os.path.join(bringup_dir, 'launch')
    risk_only_params = os.path.join(bringup_dir, 'params', 'nav2_params_dwb_risk_only.yaml')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace',
        ),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Whether to apply a namespace to the navigation stack',
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='False',
            description='Whether run a SLAM',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(bringup_dir, 'map', 'test.yaml'),
            description='Full path to map yaml file to load',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack',
        ),
        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees',
                'navigate_w_replanning_and_recovery.xml',
            ),
            description='Full path to the behavior tree xml file to use',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': use_namespace,
                'slam': slam,
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'params_file': risk_only_params,
                'default_bt_xml_filename': default_bt_xml_filename,
                'autostart': autostart,
            }.items(),
        ),
    ])
