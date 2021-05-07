import sys
from interfaces.srv import Interpolation
import rclpy
from rclpy.node import Node


class JintClient(Node):

    def __init__(self):
        super().__init__('jint_client')

        self.client = self.create_client(Interpolation, 'interpolacja')
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Oczekiwanie na serwis')
        self.request = Interpolation.Request()

    def send_request(self):
        try:
            self.request.joint1_goal = float(sys.argv[1])
            self.request.joint2_goal= float(sys.argv[2])
            self.request.joint3_goal = float(sys.argv[3])
            self.request.time = float(sys.argv[4])
            self.request.type = (sys.argv[5])

        except IndexError:
            print("Niepoprawna liczba parametrow")
            raise IndexError()
        except ValueError:
            print("Bledne parametry")
            raise ValueError()
            
        self.future = self.client.call_async(self.request)



def main(args=None):
    rclpy.init(args=args)

    try:
        client = JintClient()
        client.send_request()
    except Exception as e:
        print("Nie udalo sie uruchomic klienta")
    else:
        while rclpy.ok():
            rclpy.spin_once(client)
            if client.future.done():
                try:
                    response = client.future.result()
                except Exception as e:
                    client.get_logger().error("Interpolacja nieudana: " + str(e))
                else:
                    client.get_logger().info(response.status)
                    return
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()