import sys
from interfaces.srv import Ointxyz
import rclpy
from rclpy.node import Node


class OintClient(Node):

    def __init__(self):
        super().__init__('oint_client')

        self.client = self.create_client(Ointxyz, 'oint_control_srv')
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Oczekiwanie na serwis')
        self.request = Ointxyz.Request()

    def send_request(self):
        try:
            self.request.x = float(sys.argv[1])
            self.request.y= float(sys.argv[2])
            self.request.z = float(sys.argv[3])

            self.request.time = float(sys.argv[7])
            self.request.type = (sys.argv[8])

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
        client = OintClient()
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