from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

class Ikin(Node):

  def __init__(self):
    rclpy.init()
    super().__init__('ikin')

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

    # obliczenia tutaj

    # tymczasowe
    joint1 = z + 0.2

    self.joint_states.position = [joint1, joint2, joint3]

    self.joint_pub.publish(self.joint_states)

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