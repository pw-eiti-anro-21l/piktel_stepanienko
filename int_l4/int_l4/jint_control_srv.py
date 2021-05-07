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

from interfaces.srv import Interpolation


class Jint(Node):


    def __init__(self):
        super().__init__('jint')
        self.srv = self.create_service(Interpolation, 'interpolacja', self.interpolation_callback)
        self.start_positions = [0.2, 0, 0]

    def interpolation_callback(self, request, response):

        if request.joint1_goal < 0:
            response.status = 'Nie udalo sie dokonac interpolacji, zla pozycja docelowa zlacza 1'
            return response
        if request.time <= 0:
            response.status = 'Nie udalo sie dokonac interpolacji, zly czas interpolacji'
            return response
        if request.type != 'linear' and request.type != 'nonlinear':
            response.status = 'Nie udalo sie dokonac interpolacji, nieznany typ interpolacji'
            return response
                                               
        self.get_logger().info('Rozpoczynanie interpolacji')

        step_time = 0.1
        total_time = request.time
        steps = math.floor(total_time/step_time)

        for i in range(1,steps+1):

            qos_profile1 = QoSProfile(depth=10)
            self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile1)
            joint_state = JointState()
            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = ['el1-el2', 'el2-el3', 'el3-tool']

            joint1_goal = request.joint1_goal
            joint2_goal = request.joint2_goal
            joint3_goal = request.joint3_goal

            # Interpolacja liniowa
            if(request.type == 'linear'):
                joint1_step = self.start_positions[0] + ((joint1_goal - self.start_positions[0])/total_time)*step_time*i
                joint2_step = self.start_positions[1] + ((joint2_goal - self.start_positions[1])/total_time)*step_time*i
                joint3_step = self.start_positions[2] + ((joint3_goal - self.start_positions[2])/total_time)*step_time*i

            joint_state.position = [float(joint1_step), float(joint2_step), float(joint3_step)]
            self.joint_pub.publish(joint_state)
            time.sleep(step_time)

        self.start_positions = [joint1_step, joint2_step, joint3_step]
        self.get_logger().info('Koniec interpolacji')
        response.status = 'Interpolacja zakonczona pomyslnie'
        return response

def main(args=None):
    rclpy.init(args=args)
    jint_service = Jint()
    rclpy.spin(jint_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()