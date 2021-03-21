# Prints information on screen about turtle movement.


# Python imports
import curses


class Screen:

    def __init__(self, params):
        self.screen = curses.initscr()
        self.malformed_params = False
        self.setup(params)

    def setup(self, params):
        curses.cbreak()
        curses.noecho()
        self.screen.keypad(1)
        if self.check_params(params) is False:
            self.print_setup(params)
        self.screen.refresh()

    # Checks if any of the parameters is set to 'q',
    # which is a reserved key
    def check_params(self, params):
        for param, param_value in params.items():
            if (param_value == 'q'):
                self.print_malformed_param_file_error()
                self.malformed_params = True
        return self.malformed_params

    def print_setup(self, params):
        self.print_usage_info()
        self.print_exit_info()
        self.print_steering_info(params)
        self.screen.refresh()

    def print_exit_info(self):
        self.screen.addstr(0, 70, "Hit 'q' to quit")

    def print_speed(self, speed):
        self.screen.addstr(5, 0, "Speed value:")
        self.screen.addstr(6, 1, "Linear:")
        self.screen.addstr(7, 4, "x = " + str(speed.linear.x))
        self.screen.addstr(8, 4, "y = " + str(speed.linear.y))
        self.screen.addstr(9, 4, "z = " + str(speed.linear.z))
        self.screen.addstr(10, 1, "Angular:")
        self.screen.addstr(11, 4, "x = " + str(speed.angular.x))
        self.screen.addstr(12, 4, "y = " + str(speed.angular.y))
        self.screen.addstr(13, 4, "z = " + str(speed.angular.z))
        self.screen.refresh()

    def print_steering_info(self, params):
        X_LAYOUT = 50
        self.screen.addstr(0, X_LAYOUT, "Steering keys:")
        self.screen.addstr(1, X_LAYOUT, "Up:" + params['up'])
        self.screen.addstr(2, X_LAYOUT, "Down:" + params['down'])
        self.screen.addstr(3, X_LAYOUT, "Left:" + params['left'])
        self.screen.addstr(4, X_LAYOUT, "Right:" + params['right'])
        self.screen.refresh()

    def print_usage_info(self):
        X_LAYOUT = 0
        self.screen.addstr(0, X_LAYOUT, "Usage info:")
        self.screen.addstr(1, X_LAYOUT, "Configure custom keys in file:")
        self.screen.addstr(2, X_LAYOUT, "<workspace_folder>/config/params.yaml")
        self.screen.refresh()

    # Closes curses
    def end(self):
        curses.endwin()

    def print_malformed_param_file_error(self):
        self.screen.addstr(0, 0, "Malformed parameters file.")
        self.screen.addstr(1, 0, "Key 'q' is reserved.")
        self.screen.addstr(2, 0, "Program will be shut down.")

    # Getter
    def get_screen(self):
        return self.screen
