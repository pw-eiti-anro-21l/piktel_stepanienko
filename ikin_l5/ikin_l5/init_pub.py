import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
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

      joint1 = 0.2
      joint2 = 0.0
      joint3 = 0.0

      odom_trans = TransformStamped()
      odom_trans.header.frame_id = 'odom'
      odom_trans.child_frame_id = 'base'
      joint_state = JointState()

      t = 0
      try:
          while rclpy.ok():
              rclpy.spin_once(self)
              
              now = self.get_clock().now()
              joint_state.header.stamp = now.to_msg()
              joint_state.name = ['el1-el2', 'el2-el3', 'el3-tool']
              joint_state.position = [joint1, joint2, joint3]
              self.joint_pub.publish(joint_state)
              self.broadcaster.sendTransform(odom_trans)

              t+=1
              if t > 30:
                self.destroy_node()
                self.get_logger().info("finished")

              loop_rate.sleep()

      except KeyboardInterrupt:
          pass

def main():
  node = StatePublisher()

if __name__ == '__main__':
  main()