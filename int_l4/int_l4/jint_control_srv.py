import rclpy
from rclpy.node import Node
import os
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from rclpy.clock import ROSClock
from sensor_msgs.msg import JointState
import time
import math

from interfaces.srv import Jint
from . import markers


class Jint_srv(Node):

    def __init__(self):
        super().__init__('jint')
        self.srv = self.create_service(Jint, 'jint_control_srv', self.service_callback)
        self.start_positions = [0.2, 0, 0]
        self.step_time = 0.1
        
        self.markers = markers.Markers()

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile)

    def service_callback(self, request, response):

        if request.joint1_goal < 0:
            response.status = 'Nie udalo sie dokonac interpolacji, zla pozycja docelowa zlacza 1'
            return response
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
        total_time = request.time
        steps = math.floor(total_time/self.step_time)

        step_move1 = (request.joint1_goal - self.start_positions[0])/steps
        step_move2 = (request.joint2_goal - self.start_positions[1])/steps
        step_move3 = (request.joint3_goal - self.start_positions[2])/steps

        cpos1 = self.start_positions[0]
        cpos2 = self.start_positions[1]
        cpos3 = self.start_positions[2]

        for i in range(steps):

            joint_state = JointState()
            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = ['el1-el2', 'el2-el3', 'el3-tool']

            cpos1 += step_move1
            cpos2 += step_move2
            cpos3 += step_move3

            joint_state.position = [float(cpos1), float(cpos2), float(cpos3)]

            self.markers.marker_set_jint(cpos1, cpos2, cpos3, i)

            self.joint_pub.publish(joint_state)
            time.sleep(self.step_time)

        self.start_positions = [cpos1, cpos2, cpos3]        

    def interpolation_spline(self, request):
        joint_state = JointState()
        total_time = request.time
        steps = math.floor(total_time/self.step_time)

        joint_goals = []
        joint_goals.append(request.joint1_goal)
        joint_goals.append(request.joint2_goal)
        joint_goals.append(request.joint3_goal)

        a = self.cal_params(self.start_positions, joint_goals, total_time)

        for i in range(steps):
            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = ['el1-el2', 'el2-el3', 'el3-tool']
            
            cpos1 = a[0][0] + a[1][0]*(i*self.step_time) + a[2][0]*(i*self.step_time)**2 + a[3][0]*(i*self.step_time)**3
            cpos2 = a[0][1] + a[1][1]*(i*self.step_time) + a[2][1]*(i*self.step_time)**2 + a[3][1]*(i*self.step_time)**3
            cpos3 = a[0][2] + a[1][2]*(i*self.step_time) + a[2][2]*(i*self.step_time)**2 + a[3][2]*(i*self.step_time)**3


            joint_state.position = [float(cpos1),float(cpos2),float(cpos3)]
            self.joint_pub.publish(joint_state)

            self.markers.marker_set_jint(cpos1, cpos2, cpos3, i)

            time.sleep(self.step_time)

        self.start_positions = [cpos1, cpos2, cpos3]

    def cal_params(self, start_values, goal_values, total_time):
        a = [[0 for j in range(3)] for i in range(4)]
        for j in range(3):
            a[0][j] = start_values[j]
            a[1][j] = 0
            a[2][j] = 3*(goal_values[j] - start_values[j])/(total_time**2)
            a[3][j] = -2*(goal_values[j] - start_values[j])/(total_time**3)
        
        return a


def main(args=None):
    rclpy.init(args=args)
    jint_service = Jint_srv()
    rclpy.spin(jint_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()