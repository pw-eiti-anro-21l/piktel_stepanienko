from math import atan, sin, cos, pi, sqrt, acos, asin, atan2
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
    self.joint_prev_pos = [0.5, 0.0, 0.0]

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


    # z axis
    joint1 = z - (self.links['base']['w'] + self.links['el1']['l'] + self.links['el2']['l']/2)

    # xy plane
    a2 = self.links['el3']['a']
    a3 = self.links['tool']['l'] + self.links['el3']['r']

    data_valid = True

    try:
      joint31 = acos((x**2+y**2-a2**2-a3**2)/(2*a2*a3))
      joint21 = -asin(a3*sin(joint31)/(sqrt(x**2+y**2))) + atan2(y, x)
      delta1 = joint31 - self.joint_prev_pos[2] + joint21 - self.joint_prev_pos[1]

      joint32 = -acos((x**2+y**2-a2**2-a3**2)/(2*a2*a3))
      joint22 = -asin(a3*sin(joint32)/(sqrt(x**2+y**2))) + atan2(y, x)
      delta2 = joint32 - self.joint_prev_pos[2] + joint22 - self.joint_prev_pos[1]

      joint33 = acos((x**2+y**2-a2**2-a3**2)/(2*a2*a3))
      joint23 = pi - asin(a3*sin(joint33)/(sqrt(x**2+y**2))) + atan2(y, x)
      delta3 = joint33 - self.joint_prev_pos[2] + joint23 - self.joint_prev_pos[1]

      joint34 = -acos((x**2+y**2-a2**2-a3**2)/(2*a2*a3))
      joint24 = pi - asin(a3*sin(joint34)/(sqrt(x**2+y**2))) + atan2(y, x)
      delta4 = joint34 - self.joint_prev_pos[2] + joint24 - self.joint_prev_pos[1]

      jmin = min(delta1, delta2, delta3, delta4)

      if jmin == delta1:
        joint2 = joint21
        joint3 = joint31
      elif jmin == delta2:
        joint2 = joint22
        joint3 = joint32
      elif jmin == delta3:
        joint2 = joint23
        joint3 = joint33
      elif jmin == delta4:
        joint2 = joint24
        joint3 = joint34


    except Exception as e:
      data_valid = False

    if data_valid:
      self.joint_states.position = [joint1, joint2, joint3]
      self.joint_pub.publish(self.joint_states)
      self.joint_prev_pos = [joint1, joint2, joint3]
    else:
      self.get_logger().error("Impossible to calculate position "
        + " x: " + str(x) 
        + " y: " + str(y)
        + " z: " + str(z))

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