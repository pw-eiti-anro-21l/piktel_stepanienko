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

        T = np.eye(4)
        T[2][3] = 0.1

        for i, joint in enumerate(dhv.keys()):

            d = dhv[joint]['d']
            theta = dhv[joint]['t']

            if i == 0:
                a = links['el1']['a']
                alpha = links['el1']['alpha']

                d += msg.position[i]
                d += links['el1']['l']
            if i == 1:
                a = links['el2']['a']
                alpha = links['el2']['alpha']

                theta = msg.position[i]
                d += links['el2']['l']
            if i == 2:
                a = links['el3']['a']
                alpha = links['el3']['alpha']

                theta = msg.position[i]


            Rotx = np.array([[1, 0, 0, 0],
                             [0, cos(alpha), -sin(alpha), 0],
                             [0, sin(alpha), cos(alpha), 0],
                             [0, 0, 0, 1]])

            Transx = np.array([[1, 0, 0, a],
                               [0, 1, 0, 0],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]])

            Rotz = np.array([[cos(theta), -sin(theta), 0, 0],
                             [sin(theta), cos(theta), 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])

            Transz = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 1, d],
                               [0, 0, 0, 1]])

            T_curr = Rotx@Transx@Rotz@Transz
            T = T @ T_curr

        T = T @ np.array([[1, 0, 0, links['tool']['l']+links['el3']['r']],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

        xyz = [T[0][3], T[1][3], T[2][3]]
        print(xyz)

        rpy = mathutils.Matrix([
            [T[0][0], T[0][1], T[0][2]],
            [T[1][0], T[1][1], T[1][2]],
            [T[2][0], T[2][1], T[2][2]]
        ])
        qua = rpy.to_quaternion()


        qos_profile = QoSProfile(depth=10)
        pose_publisher = self.create_publisher(PoseStamped, '/pose_stamped_nonkdl', qos_profile)

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