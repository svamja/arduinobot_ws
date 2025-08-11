import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import AddTwoInts
import sys

class SimpleServiceClient(Node):
    
    def __init__(self, a, b):
        super().__init__("simple_service_client")
        self.client_ = self.create_client(AddTwoInts, "add_two_ints")
        self.get_logger().info("Client for 'add_two_ints' service is ready.")
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        self.request_ = AddTwoInts.Request()
        self.request_.a = a
        self.request_.b = b
        self.future_ = self.client_.call_async(self.request_)
        self.future_.add_done_callback(self.handle_response)
      
    def handle_response(self, future):
        response = future.result()
        self.get_logger().info("Response received: %d" % (response.sum))

def main():
    rclpy.init()
    if len(sys.argv) != 3:
        print("Usage: simple_service_client.py <int_a> <int_b>")
        sys.exit(1)
    a = int(sys.argv[1])
    b = int(sys.argv[2])
    simple_service_client = SimpleServiceClient(a, b)
    rclpy.spin(simple_service_client)
    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
