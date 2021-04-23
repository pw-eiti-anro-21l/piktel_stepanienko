from math import sin, cos, pi
import os
import rclpy
import yaml
import numpy as np
import mathutils
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, TransformStamped
from rclpy.clock import ROSClock

from PyKDL import *
# from PyKDL import Chain, Joint, Frame, Rotation, Vector, Segment, ChainFkSolverPos_recursive, JntToCart

class NONKDL_DKIN(Node):

    def __init__(self):
        super().__init__('NONKDL_DKIN')

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback, 10)

    def listener_callback(self, msg):

        dhv = readDH()
        dhv.pop('fixed_joints')

        links = readLinks()
        xyz_rpy = readXYZ_RPY()
        
        # Lancuch kinematyczny
        chain = Chain()

        # Joint el1-el2
        el1_el2 = Joint(Joint.TransZ)
        frame1 = Frame(
            Rotation.RPY(
                xyz_rpy['el1-el2']['roll'],
                xyz_rpy['el1-el2']['pitch'],
                xyz_rpy['el1-el2']['yaw'],
                ),
            Vector(
                xyz_rpy['el1-el2']['x'],
                xyz_rpy['el1-el2']['y'],
                xyz_rpy['el1-el2']['z'],
            )
        )
        segment1 = Segment(el1_el2, frame1)
        chain.addSegment(segment1)
        
        # Joint el2-el3
        el2_el3 = Joint(Joint.RotZ)
        frame2 = Frame(
            Rotation.RPY(
                xyz_rpy['el2-el3']['roll'],
                xyz_rpy['el2-el3']['pitch'],
                xyz_rpy['el2-el3']['yaw']
                ),
            Vector(
                xyz_rpy['el2-el3']['x'],
                xyz_rpy['el2-el3']['y'],
                xyz_rpy['el2-el3']['z'],
            )
        )
        segment2 = Segment(el2_el3, frame2)
        chain.addSegment(segment2)

        # Joint el3-tool
        el3_tool = Joint(Joint.RotX)
        frame3 = Frame(
            Rotation.RPY(
                xyz_rpy['el3-tool']['roll'],
                xyz_rpy['el3-tool']['pitch'],
                xyz_rpy['el3-tool']['yaw']
                ),
            Vector(
                xyz_rpy['el3-tool']['x'],
                xyz_rpy['el3-tool']['y'],
                xyz_rpy['el3-tool']['z'],
            )
        )
        segment3 = Segment(el3_tool, frame3)
        chain.addSegment(segment3)
        

            # Forward kinematics
        joint_positions = JntArray(3)
        joint_positions[0] = msg.position[0]
        joint_positions[1] = -msg.position[1]
        joint_positions[2] = -msg.position[2]

        # Solver
        fk = ChainFkSolverPos_recursive(chain)
        endFrame = Frame()
        fk.JntToCart(joint_positions,endFrame)

        qua = endFrame.M.GetQuaternion()
        tool_offset = Vector(
            links['tool']['aa'] + links['el3']['r'],
            0,
            0)
        endFrame.p + tool_offset
        xyz = endFrame.p
        print(xyz)


        qos_profile = QoSProfile(depth=10)
        pose_publisher = self.create_publisher(PoseStamped, '/pose_stamped_kdl', qos_profile)

        pose = PoseStamped()
        pose.header.stamp = ROSClock().now().to_msg()
        pose.header.frame_id = "base"

        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        pose.pose.orientation = Quaternion(w=qua[0], x=qua[1], y=qua[2], z=qua[3])

        pose_publisher.publish(pose)

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

    nonkdl = NONKDL_DKIN()
    try:
        rclpy.spin(nonkdl)
    except KeyboardInterrupt:
        nonkdl.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()