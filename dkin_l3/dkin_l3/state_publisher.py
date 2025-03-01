#! /usr/bin/env python
from math import sin, cos, pi
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

  def __init__(self):
      rclpy.init()
      super().__init__('state_publisher')

      qos_profile = QoSProfile(depth=10)
      self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
      self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
      self.nodeName = self.get_name()
      self.get_logger().info("{0} started".format(self.nodeName))

      loop_rate = self.create_rate(30)

      # robot state
      joint1 = 0.2
      joint2 = 0.0
      joint3 = 0.0

      joint1c = 0.001
      joint2c = 0.02
      joint3c = 0.02

      joint1min = 0.2
      joint2min = -1.0
      joint3min = -1.0

      joint1max = 0.8
      joint2max = 1.0
      joint3max = 1.0

      # message declarations
      odom_trans = TransformStamped()
      odom_trans.header.frame_id = 'odom'
      odom_trans.child_frame_id = 'base'
      joint_state = JointState()

      try:
          while rclpy.ok():
              rclpy.spin_once(self)
              
              # update joint_state
              now = self.get_clock().now()
              joint_state.header.stamp = now.to_msg()
              joint_state.name = ['el1-el2', 'el2-el3', 'el3-tool']
              joint_state.position = [joint1, joint2, joint3]

              joint1 += joint1c
              joint2 += joint2c
              joint3 += joint3c

              if joint1 >= joint1max or joint1 <= joint1min:
                joint1c *= -1

              if joint2 >= joint2max or joint2 <= joint2min:
                joint2c *= -1

              if joint3 >= joint3max or joint3 <= joint3min:
                joint3c *= -1
              
              # send the joint state and transform
              self.joint_pub.publish(joint_state)
              self.broadcaster.sendTransform(odom_trans)

              # This will adjust as needed per iteration
              loop_rate.sleep()

      except KeyboardInterrupt:
          pass
 
def euler_to_quaternion(roll, pitch, yaw):
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
  return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
  node = StatePublisher()

if __name__ == '__main__':
  main()