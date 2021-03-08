# Sets speed of the turtle according to user input.


# Python imports
import curses

# ROS2 imports
from geometry_msgs.msg import Twist

# Project imports
from turtle_manipulation.Constants import Constants


class GetSpeed:

    def __init__(self):
        self.speed = Twist()
        self.constants = Constants()

    # Runs the main loop in which keypresses are detected.
    def filter_and_set_keys(self, params, screen):
        self.reset_speed()
        loop = True
        break_loop = False
        while loop:
            loop = False
            key = screen.getch()
            if key == ord(params['up']):
                self.speed.linear.x = self.constants.L_SCALE * 1.0
            elif key == ord(params['down']):
                self.speed.linear.x = self.constants.L_SCALE * -1.0
            elif key == ord(params['left']):
                self.speed.angular.z = self.constants.A_SCALE * 1.0
            elif key == ord(params['right']):
                self.speed.angular.z = self.constants.A_SCALE * -1.0
            elif key == ord('q'):
                break_loop = True
            else:
                loop = True

        return break_loop

    # Resets the speed. Otherwise turtle would never stop
    def reset_speed(self):
        self.speed.linear.x = 0.0
        self.speed.linear.y = 0.0
        self.speed.linear.z = 0.0
        self.speed.angular.x = 0.0
        self.speed.angular.y = 0.0
        self.speed.angular.z = 0.0

    # Getter
    def get_speed(self):
        return self.speed
