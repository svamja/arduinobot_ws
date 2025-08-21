import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from arduinobot_msgs.action import Fibonacci

class SimpleActionClient(Node):

    def __init__(self, order):
        super().__init__("simple_action_client")
        self.action_client = ActionClient(self, Fibonacci, "fibonacci")
        self.action_client.wait_for_server()
        self.goal = Fibonacci.Goal()
        self.goal.order = order
        self.future = self.action_client.send_goal_async(self.goal, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.goal_response_callback)


    def feedback_callback(self, feedback_msg):
        self.get_logger().info("Feedback received: %s" % (feedback_msg.feedback.partial_sequence))

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected!")
            return
        self.get_logger().info("Goal accepted!")
        self.future = goal_handle.get_result_async()
        self.future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result received: %s" % (result.sequence))
        rclpy.shutdown()

def main():
    rclpy.init()
    simple_action_client = SimpleActionClient(10)
    rclpy.spin(simple_action_client)
    # simple_action_client.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()
