import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_file_name = 'manip.rviz'
    urdf_file_name = 'manip.urdf.'

    print('urdf_file_name : {}'.format(urdf_file_name))

    rviz = os.path.join(
        get_package_share_directory('int_l4'),
        rviz_file_name)
    urdf = os.path.join(
        get_package_share_directory('int_l4'),
        urdf_file_name)

    yaml_file_name = 'joints.yaml'
    yaml = os.path.join(get_package_share_directory('int_l4'), yaml_file_name)

    os.system('xacro '+urdf+'xacro > '+urdf+'xml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='int_l4',
            executable='XYZ_RPY',
            name='XYZ_RPY',
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf+'xml']),
        Node(
            package='int_l4',
            executable='initpub',
            name='initpub',
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='manip_rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz]),
    ])