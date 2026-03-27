import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_type = LaunchConfiguration('hardware_type')
    namespace = LaunchConfiguration('namespace')
    usb_port = LaunchConfiguration('usb_port')
    joint_config_file = LaunchConfiguration('joint_config_file')
    config_file = LaunchConfiguration('config_file')

    # MoveIt bringup (follower_split + move_group + RViz)
    follower_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('so101_bringup'),
                'launch',
                'follower_moveit_demo.launch.py',
            )
        ),
        launch_arguments={
            'hardware_type': hardware_type,
            'namespace': namespace,
            'usb_port': usb_port,
            'joint_config_file': joint_config_file,
        }.items(),
    )

    # Perception (YOLO + RealSense)
    detect_node = Node(
        package='so101_pick_place',
        executable=LaunchConfiguration('perception_executable'),
        name=LaunchConfiguration('perception_node_name'),
        output='screen',
        parameters=[config_file],
    )

    # Pick & Place
    pick_place_node = Node(
        package='so101_pick_place',
        executable='pick_place',
        name='pick_place',
        output='screen',
        remappings=[
            ('/joint_states', ['/', namespace, '/joint_states']),
            ('/robot_description', ['/', namespace, '/robot_description']),
            ('/robot_description_semantic', ['/', namespace, '/robot_description_semantic']),
        ],
        parameters=[config_file],
    )

    return LaunchDescription([
        # Hardware
        DeclareLaunchArgument('hardware_type', default_value='real'),
        DeclareLaunchArgument('namespace', default_value='follower'),
        DeclareLaunchArgument('usb_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument(
            'joint_config_file',
            default_value=os.path.join(
                get_package_share_directory('so101_pick_place'),
                'config',
                'follower_joints.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                get_package_share_directory('so101_pick_place'),
                'config',
                'pick_place.yaml',
            ),
        ),
        DeclareLaunchArgument('perception_executable', default_value='detect_and_locate'),
        DeclareLaunchArgument('perception_node_name', default_value='detect_and_locate'),

        follower_moveit,
        detect_node,
        pick_place_node,
    ])
