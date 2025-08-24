import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from arduinobot_msgs.action import Fibonacci
import time

class MyTaskServer(Node):
    
    def __init__(self):
        super().__init__("my_task_server")
        self.action_server = ActionServer(self, Fibonacci, "fibonacci", self.goalCallback)
        self.get_logger().info("Action server 'fibonacci' is ready to receive requests.")


    def goalCallback(self, goal_handle):
        self.get_logger().info("Received goal request.")
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1])
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result



def main():
    rclpy.init()
    my_task_server = MyTaskServer()
    rclpy.spin(my_task_server)
    my_task_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
