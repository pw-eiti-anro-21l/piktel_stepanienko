from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
                package='urdf_l2',
                executable='state_publisher',
                name='state_publisher',
                output='screen'),
    ])