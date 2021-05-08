import rclpy
from rclpy.node import Node

from interfaces.srv import Jint


class Jcmd(Node):
  def __init__(self):
    super().__init__('Jcmd')

    self.client = self.create_client(Jint, 'jint_control_srv')
    while not self.client.wait_for_service(timeout_sec=2.0):
      self.get_logger().info('Oczekiwanie na serwis')
    self.request = Jint.Request()
    self.def_requests()

  def send_request(self, joint1_goal, joint2_goal, joint3_goal, time, int_type):
    try:
      self.request.joint1_goal = joint1_goal
      self.request.joint2_goal= joint2_goal
      self.request.joint3_goal = joint3_goal
      self.request.time = time
      self.request.type = int_type

    except IndexError:
      print("Niepoprawna liczba parametrow")
      raise IndexError()
    except ValueError:
      print("Bledne parametry")
      raise ValueError()
          
    self.future = self.client.call_async(self.request)

  def def_requests(self):
    self.requests = [
      dict(joint1_goal = 0.8, joint2_goal = 0, joint3_goal = 0, time = 2, type = 'linear'),
      dict(joint1_goal = 0.8, joint2_goal = 1, joint3_goal = 0, time = 2, type = 'linear'),
      dict(joint1_goal = 0.8, joint2_goal = -1, joint3_goal = 0, time = 2, type = 'linear'),
      dict(joint1_goal = 0.8, joint2_goal = -1, joint3_goal = 1, time = 2, type = 'linear'),
      dict(joint1_goal = 0.8, joint2_goal = -1, joint3_goal = -1, time = 2, type = 'linear'),
      dict(joint1_goal = 0.2, joint2_goal = 0, joint3_goal = 0, time = 2, type = 'linear'),
    ]

  def run_request(self, request):
    self.send_request(
      float(request["joint1_goal"]),
      float(request["joint2_goal"]),
      float(request["joint3_goal"]),
      float(request["time"]),
      request["type"])
    while rclpy.ok():
      rclpy.spin_once(self)
      if self.future.done():
        try:
          response = self.future.result()
        except Exception as e:
          self.get_logger().error("Interpolacja nieudana: " + str(e))
          return
        else:
          self.get_logger().info(response.status)
          return

  def run_set_of_requests(self):
    while True:
      try:
        for request in self.requests:
          self.run_request(request)
      except KeyboardInterrupt:
        # Back to starting position
        self.run_request(self.requests[len(self.requests) - 1])
        self.get_logger().info("Koniec pracy automatycznego klienta.")
        return

def main(args=None):
  rclpy.init(args=args)
  try:
    client = Jcmd()
    client.run_set_of_requests()
  except Exception as e:
    print(e)
    print("Nie udalo sie uruchomic klienta")
  finally:
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



