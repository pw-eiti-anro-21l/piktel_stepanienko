import rclpy
from rclpy.node import Node
import os
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from rclpy.clock import ROSClock
from sensor_msgs.msg import JointState
import time
import math  

from interfaces.srv import Oint


class Oint_srv(Node):


    def __init__(self):
        super().__init__('minimal_client_async')
        self.srv = self.create_service(Oint, 'oint_control_srv', self.service_callback)
        self.start_positions = [0.2, 0, 0]

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile)

    def service_callback(self, request, response):

        if request.time <= 0:
            response.status = 'Nie udalo sie dokonac interpolacji, zly czas interpolacji'
            return response
        if request.type != 'linear' and request.type != 'spline':
            response.status = 'Nie udalo sie dokonac interpolacji, nieznany typ interpolacji'
            return response
                                               
        self.get_logger().info('Rozpoczynanie interpolacji')

        if request.type == 'linear':
            self.interpolation_linear(request)
            
        if request.type == 'spline':
            self.interpolation_spline(request)
    
        self.get_logger().info('Koniec interpolacji')

        response.status = 'Interpolacja zakonczona pomyslnie'
        return response

    def interpolation_linear(self, request):
        pass

    def interpolation_spline(self, request):
        pass

def main(args=None):
    rclpy.init(args=args)
    oint_service = Oint_srv()
    rclpy.spin(oint_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()