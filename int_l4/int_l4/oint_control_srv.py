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
# import transforms3d
from transforms3d.euler import euler2quat

from interfaces.srv import Oint


class Oint_srv(Node):


    def __init__(self):
        super().__init__('minimal_client_async')
        self.srv = self.create_service(Oint, 'oint_control_srv', self.service_callback)
        self.start_positions = [0, 0, 0]
        self.start_orientations = [0, 0, 0]

        qos_profile = QoSProfile(depth=10)
        self.pose_pub = self.create_publisher(PoseStamped, '/oint_pose', qos_profile)
        self.positions = PoseStamped()

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
        step_time = 0.1
        total_time = request.time
        steps = math.floor(total_time/step_time)

        step_move1 = (request.x - self.start_positions[0])/steps
        step_move2 = (request.y - self.start_positions[1])/steps
        step_move3 = (request.z - self.start_positions[2])/steps

        cpos1 = self.start_positions[0]
        cpos2 = self.start_positions[1]
        cpos3 = self.start_positions[2]

        step_rot1 = (request.roll - self.start_orientations[0])/steps
        step_rot2 = (request.pitch - self.start_orientations[1])/steps
        step_rot3 = (request.yaw - self.start_orientations[2])/steps

        rpos1 = self.start_orientations[0]
        rpos2 = self.start_orientations[1]
        rpos3 = self.start_orientations[2]

        for i in range(steps):

            now = self.get_clock().now()
            self.positions.header.stamp = now.to_msg()
            self.positions.header.frame_id = "map"

            cpos1 += step_move1
            cpos2 += step_move2
            cpos3 += step_move3

            rpos1 += step_rot1
            rpos2 += step_rot2
            rpos3 += step_rot3

            self.positions.pose.position.x = float(cpos1)
            self.positions.pose.position.y = float(cpos2)
            self.positions.pose.position.z = float(cpos3)

            self.positions.pose.orientation = self.from_euler_to_qua(rpos1, rpos2, rpos3)

            # self.marker_set(cpos1, cpos2, cpos3, i)

            self.pose_pub.publish(self.positions)
            time.sleep(step_time)

        self.start_positions = [cpos1, cpos2, cpos3]
        self.start_orientations = [rpos1, rpos2, rpos3]     


    def interpolation_spline(self, request):
        pass

    def from_euler_to_qua(self, rpos1, rpos2, rpos3):
        qua = euler2quat(rpos1, rpos2, rpos3, axes='sxyz')
        return Quaternion(w=qua[0], x=qua[1], y=qua[2], z=qua[3])


def main(args=None):
    rclpy.init(args=args)
    oint_service = Oint_srv()
    rclpy.spin(oint_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()