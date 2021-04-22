import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    yaml_file_name = 'joints.yaml'
    yaml = os.path.join(get_package_share_directory('dkin_l3'), yaml_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='dkin_l3',
            executable='KDL_DKIN',
            name='kdl_pose_stamped',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_file': yaml
            }],
            output='screen'),

    ])