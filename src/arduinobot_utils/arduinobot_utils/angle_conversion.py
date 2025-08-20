#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import EulerToQuaternion, QuaternionToEuler
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class AngleConverter(Node):
    
    def __init__(self):
        super().__init__("angle_converter")
        self.e2q_service_ = self.create_service(EulerToQuaternion, "euler_to_quaternion", self.e2qCallback )
        self.q2e_service_ = self.create_service(QuaternionToEuler, "quaternion_to_euler", self.q2eCallback )
        self.get_logger().info("Angle conversion services are ready.")

    def e2qCallback(self, request, response):
        self.get_logger().info("Incoming request: Euler angles (roll, pitch, yaw): (%f, %f, %f)" % (request.roll, request.pitch, request.yaw))
        q = quaternion_from_euler(request.roll, request.pitch, request.yaw)
        (response.x, response.y, response.z, response.w) = q
        return response
    
    def q2eCallback(self, request, response):
        self.get_logger().info("Incoming request: Quaternion (x, y, z, w): (%f, %f, %f, %f)" % (request.x, request.y, request.z, request.w))
        euler = euler_from_quaternion([request.x, request.y, request.z, request.w])
        (response.roll, response.pitch, response.yaw) = euler
        return response

def main():
    rclpy.init()
    angle_converter = AngleConverter()
    rclpy.spin(angle_converter)
    angle_converter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

