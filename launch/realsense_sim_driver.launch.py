from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_file = PathJoinSubstitution([
        FindPackageShare('realsense_sim_driver'),
        'config',
        'config.yaml',
    ])

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the realsense_sim_driver ROS parameter YAML file.',
    )

    node = Node(
        package='realsense_sim_driver',
        executable='aligned_realsense_node',
        name='realsense_sim_driver',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([config_file_arg, node])
