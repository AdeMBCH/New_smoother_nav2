from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    goals_file = LaunchConfiguration('goals_file')
    output_dir = LaunchConfiguration('output_dir')

    return LaunchDescription([
        DeclareLaunchArgument('goals_file', default_value=''),
        DeclareLaunchArgument('output_dir', default_value='benchmark_results'),
        Node(
            package='nav2_se2_hybrid_benchmark',
            executable='run_benchmark',
            output='screen',
            arguments=['--output-dir', output_dir, '--goals-file', goals_file],
        ),
    ])
