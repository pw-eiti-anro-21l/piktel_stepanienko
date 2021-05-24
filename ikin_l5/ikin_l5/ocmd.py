import sys
import rclpy
from rclpy.node import Node
import math
import matplotlib.pyplot as plt

from interfaces.srv import Ointxyz

class Ocmd(Node):
  def __init__(self):
    super().__init__('Ocmd')
    self.a = 0.6
    self.b = 0.4
    self.ellipse_points_number = 30
    self.ellipse_time = 0.4
    self.ellipse_x = []
    self.ellipse_y = []
    self.figures = []
    self.square = []
    self.ellipse = []


    self.client = self.create_client(Ointxyz, 'oint_control_srv')
    while not self.client.wait_for_service(timeout_sec=2.0):
      self.get_logger().info('Oczekiwanie na serwis')
    self.request = Ointxyz.Request()
    self.def_requests()

  def def_requests(self):

    self.square = self.create_square_horizontal_request()
    self.ellipse_horizontal = self.create_ellipse_horizontal_request()
    self.ellipse_vertical = self.create_ellipse_vertical_request()

    self.figures.append(self.square)
    self.figures.append(self.ellipse_horizontal)
    self.figures.append(self.ellipse_vertical)

  def create_square_horizontal_request(self):
    square = [
      dict(x = 0.4, y = 0.4, z = 0.8, time = 3, int_type = sys.argv[1]),
      dict(x = 0.4, y = -0.4, z = 0.8, time = 3, int_type = sys.argv[1]),
      dict(x = -0.4, y = -0.4, z = 0.8, time = 3, int_type = sys.argv[1]),
      dict(x = -0.4, y = 0.4, z = 0.8, time = 3, int_type = sys.argv[1]),
      dict(x = 0.4, y = 0.4, z = 0.8, time = 3, int_type = sys.argv[1])
    ]

    return square

  def create_ellipse_horizontal_request(self):
    self.cal_ellipse(self.ellipse_points_number, self.a, self.b)
    self.ellipse_horizontal = []

    # self.ellipse_horizontal.append(dict(x = self.a, y = 0, z = 0.8, time = 2, int_type = sys.argv[1]))

    for ellipse_x, ellipse_y in zip(self.ellipse_x, self.ellipse_y):
      self.ellipse_horizontal.append(dict(x = ellipse_x, y = ellipse_y, z = 0.8, time = self.ellipse_time, int_type = sys.argv[1]))
    
    return self.ellipse_horizontal

  def create_ellipse_vertical_request(self):
    self.cal_ellipse(self.ellipse_points_number, 0.5, 0.2)

    self.add_robot_offset_to_ellipse()
    self.ellipse_vertical = []

    self.ellipse_vertical.append(dict(x = 0, y = self.a, z = 0.8, time = 2, int_type = sys.argv[1]))

    for ellipse_x, ellipse_y in zip(self.ellipse_x, self.ellipse_y):
      new_x = math.sqrt(abs(0.5*0.5 - ellipse_x*ellipse_x))
      self.ellipse_vertical.append(dict(x = new_x, y = ellipse_x, z = ellipse_y, time = self.ellipse_time, int_type = sys.argv[1]))
    
    return self.ellipse_vertical

  # ellipse major (a) and minor (b) axis parameters
  def cal_ellipse(self, n, a, b):

    # num points for transformation lookup function
    npoints = 1000
    delta_theta=2.0*math.pi/npoints

    theta=[0.0]
    delta_s=[0.0]
    integ_delta_s=[0.0]

    # integrated probability density
    integ_delta_s_val=0.0

    for iTheta in range(1,npoints+1):
      # ds/d(theta):
      delta_s_val=math.sqrt(a**2*math.sin(iTheta*delta_theta)**2+ \
                            b**2*math.cos(iTheta*delta_theta)**2)

      theta.append(iTheta*delta_theta)
      delta_s.append(delta_s_val)
      # do integral
      integ_delta_s_val = integ_delta_s_val+delta_s_val*delta_theta
      integ_delta_s.append(integ_delta_s_val)
      
    # normalize integrated ds/d(theta) to make into a scaled CDF (scaled to 2*pi)
    integ_delta_s_norm = []
    for iEntry in integ_delta_s:
      integ_delta_s_norm.append(iEntry/integ_delta_s[-1]*2.0*math.pi)    

    # Create corrected ellipse using lookup function
    ellip_x_prime=[]
    ellip_y_prime=[]

    npoints_new = n
    delta_theta_new=2*math.pi/npoints_new

    for theta_index in range(npoints_new):
      theta_val = theta_index*delta_theta_new
      
      # Do lookup:
      for lookup_index in range(len(integ_delta_s_norm)):
        if theta_val >= integ_delta_s_norm[lookup_index] and theta_val < integ_delta_s_norm[lookup_index+1]:
          theta_prime=theta[lookup_index]
          break
        
      # ellipse with transformation applied
      ellip_x_prime.append(a*math.cos(theta_prime))
      ellip_y_prime.append(b*math.sin(theta_prime))
    
    self.ellipse_x = ellip_x_prime
    self.ellipse_y = ellip_y_prime

  def add_robot_offset_to_ellipse(self):
    for y, i in zip(self.ellipse_y, range(len(self.ellipse_y))):
      self.ellipse_y[i] = y + 0.2 + 0.4 + 0.2

  def send_request(self, x, y, z, time, int_type):
    try:
      self.request.x = x
      self.request.y = y
      self.request.z = z

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
        self.choose_figure()
        # for figure in self.figures:
        #   for request in figure:
        #     self.run_request(request)
      except KeyboardInterrupt:
        # Back to starting position
        self.run_request(self.figures[0][0])
        self.get_logger().info("Koniec pracy automatycznego klienta.")
        return

  def choose_figure(self):
    if sys.argv[2] == 'all':
      for figure in self.figures:
        for request in figure:
          self.run_request(request)
    elif sys.argv[2] == 'square':
      for request in self.figures[0]:
        self.run_request(request)
    elif sys.argv[2] == 'h_ellipse':
      for request in self.figures[1]:
        self.run_request(request)
    elif sys.argv[2] == 'v_ellipse':
      for request in self.figures[2]:
        self.run_request(request)
    else:
      raise ValueError

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
    print("Niepoprawny typ interpolacji lub figura.")
  except Exception as e:
    print(e)
    print("Nie udalo sie uruchomic klienta")
  finally:
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
