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

class KDL_DKIN(Node):

    def __init__(self):
        super().__init__('KDL_DKIN')

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback, 10)

    def listener_callback(self, msg):

        # Data load
        dhv = readDH()
        dhv.pop('fixed_joints')
        links = readLinks()
        xyz_rpy = readXYZ_RPY()

        tool_offset = Vector(
        links['tool']['l'] + links['el3']['r'], 0, 0)

        # Lancuch kinematyczny
        chain = Chain()

        # Joint base-el1
        base_el1 = Joint(Joint.Fixed)
        frame0 = Frame(
            Rotation.RPY(
                xyz_rpy['base-el1']['roll'],
                xyz_rpy['base-el1']['pitch'],
                xyz_rpy['base-el1']['yaw'],
                ),
            Vector(
                xyz_rpy['base-el1']['x'],
                xyz_rpy['base-el1']['y'],
                xyz_rpy['base-el1']['z'],
            )
        )
        segment0 = Segment(base_el1, frame0)
        chain.addSegment(segment0)

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

        # Joint el3_z_axis
        # Sets the rotation axis
        el3_z_axis = Joint(Joint.Fixed)
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

        [joint_positions[0],
        joint_positions[1],
        joint_positions[2]] = self.check_joint_limits(
            msg.position[0],
            msg.position[1],
            msg.position[2])

        # Solver
        fk = ChainFkSolverPos_recursive(chain)
        endFrame = Frame()
        fk.JntToCart(joint_positions,endFrame)
        qua = endFrame.M.GetQuaternion()

        xyz = endFrame.p

        # Log current position with 3 decimal numbers
        self.get_logger().info("Current xyz position: [" +
        "{:.3f}".format(xyz.x()) + ", " +
        "{:.3f}".format(xyz.y()) + ", " +
        "{:.3f}".format(xyz.z()) + "]")

        qos_profile = QoSProfile(depth=10)
        pose_publisher = self.create_publisher(PoseStamped, '/pose_stamped_kdl', qos_profile)

        pose = PoseStamped()
        pose.header.stamp = ROSClock().now().to_msg()
        pose.header.frame_id = "base"

        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        pose.pose.orientation = Quaternion(w=qua[3], x=qua[0], y=qua[1], z=qua[2])

        pose_publisher.publish(pose)

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

    kdl = KDL_DKIN()
    try:
        rclpy.spin(kdl)
    except KeyboardInterrupt:
        kdl.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()