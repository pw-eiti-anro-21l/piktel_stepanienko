# Basic publisher for topic cmd_vel.
# Publishes messages of type geometry_msgs/msg/Twist.


# Python imports:
from os import system
from time import sleep

# ROS2 imports:
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from geometry_msgs.msg import Twist

# Project imports:
from turtle_manipulation.GetSpeed import GetSpeed
from turtle_manipulation.Screen import Screen


class Move(Node):
    def __init__(self):
        super().__init__('Move')

        self.turtle = '/turtle1'
        self.topic = '/cmd_vel'
        self.params = {}

        # Create publisher
        self.publisher_ = self.create_publisher(Twist, self.turtle + self.topic, 10)
        # Create timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish)
        # Declare parameters
        self.declare_parameters(namespace = '', parameters = [
            ('up', 'g'),
            ('down', 'd'),
            ('right', 'p'),
            ('left', 'l')
        ])
        self.intialise_params()
        self.get_speed = GetSpeed()
        self.screen = Screen(self.params)
        self.check_params()

    # Checks if any of the params got 'q' assigned
    def check_params(self):
        if self.screen.malformed_params:
            sleep(5)
            self.close()
        else:
            pass

    def update_screen(self):
        break_loop = self.get_speed.filter_keys_and_set_speed(self.params, self.screen.get_screen())
        # Prints current speed on screen
        self.screen.print_speed(self.get_speed.get_speed())
        return break_loop

    def publish(self):
        break_loop = self.update_screen()
        # Publisches the speed
        self.publisher_.publish(self.get_speed.get_speed())
        self.check_end(break_loop)

    # Overrides the base parameter values.
    def intialise_params(self):
        self.params['up'] = self.get_parameter('up')._value
        self.params['down'] = self.get_parameter('down')._value
        self.params['left'] = self.get_parameter('left')._value
        self.params['right'] = self.get_parameter('right')._value

    def check_end(self, break_loop):
        if break_loop:
            self.close()
        else:
            pass

    def close(self):
        system("killall turtlesim_node")
        self.screen.end()
        system("killall move")


def main(args=None):
    rclpy.init(args=args)

    move = Move()

    # ROS2 node launch
    rclpy.spin(move)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
