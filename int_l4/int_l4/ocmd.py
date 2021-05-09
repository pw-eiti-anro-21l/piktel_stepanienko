import sys
import rclpy
from rclpy.node import Node
import math

from interfaces.srv import Oint

class Ocmd(Node):
  def __init__(self):
    super().__init__('Ocmd')

    self.client = self.create_client(Oint, 'oint_control_srv')
    while not self.client.wait_for_service(timeout_sec=2.0):
      self.get_logger().info('Oczekiwanie na serwis')
    self.request = Oint.Request()
    self.def_requests()

  def def_requests(self):
    self.requests = [
      dict(x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = math.pi/2, time = 2, int_type = sys.argv[1]),
      dict(x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0, time = 2, int_type = sys.argv[1]),
      dict(x = 0, y = 0, z = 0, roll = 0, pitch = math.pi/2, yaw = 0, time = 2, int_type = sys.argv[1]),
      dict(x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0, time = 2, int_type = sys.argv[1]),
      dict(x = 0, y = 0, z = 0, roll = math.pi/2, pitch = 0, yaw = 0, time = 2, int_type = sys.argv[1]),
      dict(x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0, time = 2, int_type = sys.argv[1]),
      dict(x = 2, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0, time = 2, int_type = sys.argv[1]),
      dict(x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0, time = 2, int_type = sys.argv[1]),
      dict(x = 0, y = 2, z = 0, roll = 0, pitch = 0, yaw = 0, time = 2, int_type = sys.argv[1]),
      dict(x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0, time = 2, int_type = sys.argv[1]),
      dict(x = 0, y = 0, z = 2, roll = 0, pitch = 0, yaw = 0, time = 2, int_type = sys.argv[1]),
      dict(x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0, time = 2, int_type = sys.argv[1]),
    ]

  def send_request(self, x, y, z, roll, pitch, yaw, time, int_type):
    try:
      self.request.x = x
      self.request.y = y
      self.request.z = z
      self.request.roll = roll
      self.request.pitch = pitch
      self.request.yaw = yaw

      self.request.time = time
      self.request.type = int_type

    except IndexError:
      print("Niepoprawna liczba parametrow")
      raise IndexError()
    except ValueError:
      print("Bledne parametry")
      raise ValueError()
          
    self.future = self.client.call_async(self.request)

  def run_request(self, request):
    self.send_request(
      float(request["x"]),
      float(request["y"]),
      float(request["z"]),
      float(request["roll"]),
      float(request["pitch"]),
      float(request["yaw"]),
      float(request["time"]),
      request["int_type"])
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
    self.check_int_type()
    while True:
      try:
        for request in self.requests:
          self.run_request(request)
      except KeyboardInterrupt:
        # Back to starting position
        self.run_request(self.requests[len(self.requests) - 1])
        self.get_logger().info("Koniec pracy automatycznego klienta.")
        return

  def check_int_type(self):
    if sys.argv[1] != 'linear' and sys.argv[1] != 'spline':
      raise ValueError
    else:
      pass

def main(args=None):
  rclpy.init(args=args)
  try:
    client = Ocmd()
    client.run_set_of_requests()
  except ValueError:
    print("Niepoprawny typ interpolacji.")
  except Exception as e:
    print(e)
    print("Nie udalo sie uruchomic klienta")
  finally:
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
