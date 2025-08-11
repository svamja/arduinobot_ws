import rclpy
from rclpy.node import Node
from arduinobot_msgs import AddTwoInts


class SimpleServiceServer(Node):
    
    def __init__(self):
        super().__init__("simple_service_server")
        self.service_ = self.create_service(
            AddTwoInts,
            "add_two_ints",
            self.handle_service_request
        )


    def handle_service_request(self, request, response):
        self.get_logger().info("Incoming request: %d + %d" % (request.a, request.b))
        response.sum = request.a + request.b
        return response


def main():
    rclpy.init()
    simple_service_server = SimpleServiceServer()
    rclpy.spin(simple_service_server)
    simple_service_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
