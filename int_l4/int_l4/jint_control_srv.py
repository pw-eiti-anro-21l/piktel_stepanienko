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
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import yaml
import time
import math

from interfaces.srv import Jint
from PyKDL import *


class Jint_srv(Node):

    def __init__(self):
        super().__init__('jint')
        self.srv = self.create_service(Jint, 'jint_control_srv', self.service_callback)
        self.start_positions = [0.2, 0, 0]

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

        self.data_init()
        self.marker_init()
                                               
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
            self.get_logger().info(str(cpos1))

            self.marker_set(cpos1, cpos2, cpos3, i)

            self.joint_pub.publish(joint_state)
            time.sleep(step_time)

        self.start_positions = [cpos1, cpos2, cpos3]        

    def interpolation_spline(self, request):
        pass

    def marker_init(self):
        self.markerArray = MarkerArray()
        qos_profile = QoSProfile(depth=10)

        self.marker_pub = self.create_publisher(MarkerArray, '/int_track', qos_profile)
        self.marker = Marker()
        self.marker.header.frame_id = "base"

        self.marker.id = 0
        self.marker.action = self.marker.DELETEALL
        self.marker_pub.publish(self.markerArray)
        self.marker.type = self.marker.SPHERE
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.04
        self.marker.scale.y = 0.04
        self.marker.scale.z = 0.04
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.orientation.x = 1.0
        self.marker.pose.orientation.y = 1.0
        self.marker.pose.orientation.z = 1.0

    def marker_set(self, cpos1, cpos2, cpos3, marker_id):
        xyz, orientation = self.get_marker_pose(cpos1, cpos2, cpos3)
        
        # New marker position values
        self.marker.pose.position.x = xyz[0]
        self.marker.pose.position.y = xyz[1]
        self.marker.pose.position.z = xyz[2]
        
        # New marker orientation values
        self.marker.pose.orientation = orientation

        self.marker.id = marker_id
        self.set_marker_colour(marker_id)
        
        # Adding new marker
        self.markerArray.markers.append(self.marker)

        # # Setting new marker id
        # id = 0
        # for marker in self.markerArray.markers:
        #     marker.id = id
        #     id += 1

        self.marker_pub.publish(self.markerArray)

    def set_marker_colour(self, marker_id):
        if marker_id % 3 == 0:
            self.marker.color.a = 1.0
            self.marker.color.r = 1.0
            self.marker.color.g = 0.0
            self.marker.color.b = 0.0
        elif marker_id % 3 == 1:
            self.marker.color.a = 1.0
            self.marker.color.r = 0.0
            self.marker.color.g = 1.0
            self.marker.color.b = 0.0
        elif marker_id % 3 == 2:
            self.marker.color.a = 1.0
            self.marker.color.r = 0.0
            self.marker.color.g = 0.0
            self.marker.color.b = 1.0

    def data_init(self):
        # self.dhv = readDH()
        # self.dhv.pop('fixed_joints')
        self.links = readLinks()
        self.xyz_rpy = readXYZ_RPY()

    def get_marker_pose(self, cpos1, cpos2, cpos3):
        tool_offset = Vector(
        self.links['tool']['l'] + self.links['el3']['r'], 0, 0)

        # Lancuch kinematyczny
        chain = Chain()

        # Joint base-el1
        base_el1 = Joint(Joint.Fixed)
        frame0 = Frame(
            Rotation.RPY(
                self.xyz_rpy['base-el1']['roll'],
                self.xyz_rpy['base-el1']['pitch'],
                self.xyz_rpy['base-el1']['yaw'],
                ),
            Vector(
                self.xyz_rpy['base-el1']['x'],
                self.xyz_rpy['base-el1']['y'],
                self.xyz_rpy['base-el1']['z'],
            )
        )
        segment0 = Segment(base_el1, frame0)
        chain.addSegment(segment0)

        # Joint el1-el2
        el1_el2 = Joint(Joint.TransZ)
        frame1 = Frame(
            Rotation.RPY(
                self.xyz_rpy['el1-el2']['roll'],
                self.xyz_rpy['el1-el2']['pitch'],
                self.xyz_rpy['el1-el2']['yaw'],
                ),
            Vector(
                self.xyz_rpy['el1-el2']['x'],
                self.xyz_rpy['el1-el2']['y'],
                self.xyz_rpy['el1-el2']['z'],
            )
        )
        segment1 = Segment(el1_el2, frame1)
        chain.addSegment(segment1)
        
        # Joint el2-el3
        el2_el3 = Joint(Joint.RotZ)
        frame2 = Frame(
            Rotation.RPY(
                self.xyz_rpy['el2-el3']['roll'],
                self.xyz_rpy['el2-el3']['pitch'],
                self.xyz_rpy['el2-el3']['yaw']
                ),
            Vector(
                self.xyz_rpy['el2-el3']['x'],
                self.xyz_rpy['el2-el3']['y'],
                self.xyz_rpy['el2-el3']['z'],
            )
        )
        segment2 = Segment(el2_el3, frame2)
        chain.addSegment(segment2)

        # Joint el3_z_axis
        # Sets the rotation axis
        el3_z_axis = Joint(Joint.Fixed)
        frame3 = Frame(
            Rotation.RPY(
                self.xyz_rpy['el3-tool']['roll'],
                self.xyz_rpy['el3-tool']['pitch'],
                self.xyz_rpy['el3-tool']['yaw']
                ),
            Vector(
                self.xyz_rpy['el3-tool']['x'],
                self.xyz_rpy['el3-tool']['y'],
                self.xyz_rpy['el3-tool']['z'],
            )
        )
        segment3 = Segment(el3_z_axis, frame3)
        chain.addSegment(segment3)
        
        # Joint el3-tool
        el3_tool = Joint(Joint.RotZ)
        frame4 = Frame(
            Rotation.RPY(0,0,0),
            tool_offset
        )
        segment4 = Segment(el3_tool, frame4)
        chain.addSegment(segment4)

        joint_positions = JntArray(3)

        # [joint_positions[0],
        # joint_positions[1],
        # joint_positions[2]] = self.check_joint_limits( cpos1, cpos2, cpos3)

        joint_positions[0] = cpos1
        joint_positions[1] = cpos2
        joint_positions[2] = cpos3

        # Solver
        fk = ChainFkSolverPos_recursive(chain)
        endFrame = Frame()
        fk.JntToCart(joint_positions,endFrame)
        qua = endFrame.M.GetQuaternion()

        xyz = endFrame.p
        orientation = Quaternion(w=qua[3], x=qua[0], y=qua[1], z=qua[2])

        # Log current position with 3 decimal numbers
        self.get_logger().info("Current xyz position: [" +
        "{:.3f}".format(xyz.x()) + ", " +
        "{:.3f}".format(xyz.y()) + ", " +
        "{:.3f}".format(xyz.z()) + "]")

        return xyz, orientation

    def check_joint_limits(self, joint1, joint2, joint3):
        joint1min = 0.2
        joint2min = -1.0
        joint3min = -1.0

        joint1max = 0.8
        joint2max = 1.0
        joint3max = 1.0

        if joint1 >= joint1max or joint1 <= joint1min:
            self.get_logger().info("ERROR: Joint el1-el2 is overstepping!")
            if joint1 >= joint1max:
                joint1_out = joint1max
            elif joint1 <= joint1min:
                joint1_out = joint1min
        else:
            joint1_out = joint1

        if joint2 >= joint2max or joint2 <= joint2min:
            self.get_logger().info("ERROR: Joint el2-el3 is overstepping!")
            if joint2 >= joint2max:
                joint2_out = joint2max
            elif joint2 <= joint2min:
                joint2_out = joint2min
        else:
            joint2_out = joint2

        if joint3 >= joint3max or joint3 <= joint3min:
            self.get_logger().info("ERROR: Joint el3-tool is overstepping!")
            if joint3 >= joint3max:
                joint3_out = joint3max
            elif joint3 <= joint3min:
                joint3_out = joint3min
        else:
            joint3_out = joint3
        
        return joint1_out, joint2_out, joint3_out

def readDH():
    with open(os.path.join(get_package_share_directory('dkin_l3'), 'joints.yaml'), 'r') as file:
        dhv = yaml.load(file, Loader=yaml.FullLoader)

    return dhv

def readLinks():
    with open(os.path.join(get_package_share_directory('dkin_l3'), 'links.yaml'), 'r') as file:
        links = yaml.load(file, Loader=yaml.FullLoader)

    return links

def readXYZ_RPY():
    with open(os.path.join(get_package_share_directory('dkin_l3'), 'xyz_rpy.yaml'), 'r') as file:
        xyz_rpy = yaml.load(file, Loader=yaml.FullLoader)

    return xyz_rpy



def main(args=None):
    rclpy.init(args=args)
    jint_service = Jint_srv()
    rclpy.spin(jint_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()