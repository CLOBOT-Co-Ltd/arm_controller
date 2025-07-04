import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node


def generate_launch_description():

    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('arm_controller'),
            'param',
            'arm_controller_param.yaml'))


    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path of arm controller parameter file'),


        Node(
            package='arm_controller',
            executable='arm_controller_node',
            name='arm_controller_node',
            parameters=[param_dir],
            arguments=['--net_if', 'eno1'],
            output='screen',
            remappings=[
            ]
        )
    ])
