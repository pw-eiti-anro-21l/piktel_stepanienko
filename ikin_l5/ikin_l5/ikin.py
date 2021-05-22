from math import sin, cos, pi
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

import yaml
import os
from ament_index_python.packages import get_package_share_directory

class Ikin(Node):

  def __init__(self):
    rclpy.init()
    super().__init__('ikin')
    self.dhv = readDH()
    self.links = readLinks()

    self.subscription = self.create_subscription(
      PoseStamped(),
      '/oint_pose',
      self.listener_callback,
      10)

    qos_profile = QoSProfile(depth=10)
    self.joint_pub = self.create_publisher(
      JointState,
      'joint_states',
      qos_profile)

    self.joint_states = JointState()

  def listener_callback(self, msg):
    now = self.get_clock().now()
    self.joint_states.header.stamp = now.to_msg()
    self.joint_states.name = ['el1-el2', 'el2-el3', 'el3-tool']

    joint1 = 0.0
    joint2 = 0.0
    joint3 = 0.0

    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    # chyba tego mozemy użyć do wyznaczenia orientacji
    fi_z = msg.pose.orientation.z

    # obliczenia tutaj

    # el1-el2
      # 0.2 - wysokość podstawy
      # 0.4 - Wyskokość pierwszego elementu
      # 0.1 - pół wysokości drugiego elementu
    joint1 = z - (self.links['base']['w'] + self.links['el1']['l'] + self.links['el2']['l']/2)

    # el2-el3
      # promień el2 i el3 (0.1) + odległość między nimi (0.5)
    a2 = self.links['el2']['r'] + self.links['el3']['r'] + self.links['el3']['a']
    joint2 = np.arctan2(y/a2, x/a2)

    self.joint_states.position = [joint1, joint2, joint3]
    self.joint_pub.publish(self.joint_states)

def readDH():
  with open(os.path.join(get_package_share_directory('ikin_l5'), 'joints.yaml'), 'r') as file:
      return yaml.load(file, Loader=yaml.FullLoader)

def readLinks():
  with open(os.path.join(get_package_share_directory('ikin_l5'), 'links.yaml'), 'r') as file:
    return yaml.load(file, Loader=yaml.FullLoader)


def main(args=None):
  try:
    node = Ikin()
    rclpy.spin(node)
  except Exception as e:
    print(e)
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()