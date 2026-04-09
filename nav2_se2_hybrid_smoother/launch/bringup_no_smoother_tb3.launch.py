from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('nav2_se2_hybrid_smoother')
    nav2_bringup_share = FindPackageShare('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'nav2_params_no_smoother.yaml'])
        ),
        DeclareLaunchArgument(
            'bt_xml_file',
            default_value=PathJoinSubstitution([pkg_share, 'bt', 'nav_to_pose_no_smoothing.xml'])
        ),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('headless', default_value='False'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([nav2_bringup_share, 'launch', 'tb3_simulation_launch.py'])
            ),
            launch_arguments={
                'params_file': params_file,
                'bt_xml_file': bt_xml_file,
                'use_sim_time': use_sim_time,
                'headless': headless,
            }.items()
        )
    ])
