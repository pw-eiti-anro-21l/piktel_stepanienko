# Launch file for turtle_manipulation package.
# Launches turtle and a new terminal with a program to control it.


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            # Launches turtle that will be controlled
            Node(
                package='turtlesim',
                executable='turtlesim_node'
            ),
            # Launches turtle manipulation programm in a new terminal
            Node(
                package='turtle_manipulation',
                executable='move',
                prefix=["gnome-terminal ", "-- "],
                output='screen',
                parameters=["./config/params.yaml"]
            )
        ]
    )