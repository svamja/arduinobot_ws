import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameter(Node):

    def __init__(self):
        super().__init__("simple_parameter")
        self.declare_parameter("my_yaw", 1.2)
        self.declare_parameter("my_pitch", 45)
        self.declare_parameter("my_color", "blue")
        self.add_on_set_parameters_callback(self.changeParamCallback)

    
    def changeParamCallback(self, params):
        result = SetParametersResult()
        result.successful = True

        for param in params:
            self.get_logger().info("param %s changed. new value = %s" % (param.name, param.value))

        return result
    

def main():
    rclpy.init()
    my_node = SimpleParameter()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()