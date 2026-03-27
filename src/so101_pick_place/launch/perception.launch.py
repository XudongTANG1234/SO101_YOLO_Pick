import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('so101_pick_place'),
        'config',
        'pick_place.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config_file),

        Node(
            package='so101_pick_place',
            executable='detect_and_locate',
            name='detect_and_locate',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
