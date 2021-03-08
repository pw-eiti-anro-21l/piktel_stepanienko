# Basic publisher for topic cmd_vel.
# Publishes messages of type geometry_msgs/msg/Twist.


# Python imports:
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

# ROS2 imports:
from geometry_msgs.msg import Twist

# Program imports:
from turtle_manipulation import GetSpeed


class Move(Node):
    def __init__(self):
        super().__init__('Move')

        self.turtle = '/turtle1'
        self.topic = '/cmd_vel'
        self.params = {}

        # Create publisher
        self.publisher_ = self.create_publisher(Twist, self.turtle + self.topic, 10)
        # Declare parameters
        self.declare_parameters(namespace = '', parameters = [
            ('up', 'g'),
            ('down', 'd'),
            ('right', 'p'),
            ('left', 'l')
        ])
        self.intialise_params()
        self.get_speed = GetSpeed.GetSpeed(self.params)

    def publish(self):
        break_loop = self.get_speed.filter_and_set_keys(self.params)
        # Prints current speed on screen
        self.get_speed.print_speed(self.get_speed.getSpeed())
        # Publisches the speed
        self.publisher_.publish(self.get_speed.getSpeed())
        return break_loop

    """
        Overrides the base parameter values.
    """
    def intialise_params(self):
        self.params['up'] = self.get_parameter('up')._value
        self.params['down'] = self.get_parameter('down')._value
        self.params['left'] = self.get_parameter('left')._value
        self.params['right'] = self.get_parameter('right')._value

    def run(self):
        break_loop = False
        while break_loop is False:
            break_loop = self.publish()
        self.get_speed.end()


def main(args=None):
    rclpy.init(args=args)

    move = Move()
    move.run()

    # ROS2 node launch
    rclpy.spin(move)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
