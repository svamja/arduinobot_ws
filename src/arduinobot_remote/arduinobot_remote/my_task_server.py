import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from arduinobot_msgs.action import ArduinobotTask
import numpy as np
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

class MyTaskServer(Node):
    
    def __init__(self):
        super().__init__("my_task_server")
        self.action_server = ActionServer(self, ArduinobotTask, "my_task_server", self.goalCallback)
        
        # Initialize MoveItPy once in the constructor
        self.get_logger().info("Initializing MoveIt...")
        self.arduinobot = MoveItPy(node_name='moveit_py')
        self.arm = self.arduinobot.get_planning_component("arm")
        self.gripper = self.arduinobot.get_planning_component("gripper")
        
        self.get_logger().info("My task server is ready to receive requests.")


    def goalCallback(self, goal_handle):
        self.get_logger().info("Goal received")

        self.get_logger().info("moving robot")
        task_number = goal_handle.request.task_number
        self.move_robot(task_number)

        goal_handle.succeed()
        result = ArduinobotTask.Result()
        result.success = True
        return result

    def move_robot(self, task_number):
        # Use the class instance variables instead of creating new ones
        arm_state = RobotState(self.arduinobot.get_robot_model())
        gripper_state = RobotState(self.arduinobot.get_robot_model())

        arm_joint_goal = []
        gripper_joint_goal = []

        if task_number == 0:
            arm_joint_goal = np.array([0.0, 0.0, 0.0])
            gripper_joint_goal = np.array([-0.7, 0.7])
        elif task_number == 1:
            arm_joint_goal = np.array([-1.14, -0.6, -0.07])
            gripper_joint_goal = np.array([0.0, 0.0])
        elif task_number == 2:
            arm_joint_goal = np.array([-1.57,0.0,-0.9])
            gripper_joint_goal = np.array([0.0, 0.0])
        else:
            self.get_logger().error("Invalid Task Number")
            return

        arm_state.set_joint_group_positions("arm", arm_joint_goal)
        gripper_state.set_joint_group_positions("gripper", gripper_joint_goal)

        self.arm.set_start_state_to_current_state()
        self.gripper.set_start_state_to_current_state()

        self.arm.set_goal_state(robot_state=arm_state)
        self.gripper.set_goal_state(robot_state=gripper_state)

        arm_plan_result = self.arm.plan()
        gripper_plan_result = self.gripper.plan()

        if arm_plan_result and gripper_plan_result:
            self.arduinobot.execute(arm_plan_result.trajectory, controllers=[])
            self.arduinobot.execute(gripper_plan_result.trajectory, controllers=[])
        else:
            self.get_logger().error('Planning failed')


def main():
    rclpy.init()
    my_task_server = MyTaskServer()
    rclpy.spin(my_task_server)
    my_task_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
