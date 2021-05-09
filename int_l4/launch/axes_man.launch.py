import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_file_name = 'axes.rviz'

    rviz = os.path.join(
        get_package_share_directory('int_l4'),
        rviz_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ['0', '0', '0', '0', '0', '0', '1', 'map', 'base']),
        Node(
            package="int_l4",
            executable='oint',
            output="screen"
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='axes_rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz]),
    ])