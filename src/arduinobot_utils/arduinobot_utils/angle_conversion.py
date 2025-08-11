#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import AddTwoInts


class AngleConverter(Node):
    
    def __init__(self):
        super().__init__("angle_converter")
        self.e2q_service_ = self.create_service( AddTwoInts, "euler_to_quaternion", self.e2qCallback )
        self.q2e_service_ = self.create_service( AddTwoInts, "quaternion_to_euler", self.q2eCallback )
        self.get_logger().info("Angle conversion services are ready.")

    def e2qCallback(self, request, response):
        self.get_logger().info("Incoming request: %d + %d" % (request.a, request.b))
        response.sum = request.a + request.b
        return response


def main():
    rclpy.init()
    angle_converter = AngleConverter()
    rclpy.spin(angle_converter)
    angle_converter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

