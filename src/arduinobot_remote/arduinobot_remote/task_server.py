import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from arduinobot_msgs.action import ArduinobotTask
import numpy as np
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

class TaskServer(Node):
    
    def __init__(self):
        super().__init__("task_server")
        self.action_server = ActionServer(self, ArduinobotTask, "arduinobot_task", self.goalCallback)
        self.get_logger().info("Task server started")
        self.arduinobot = MoveItPy(node_name='moveit_py')
        self.arduinobot_arm = self.arduinobot.get_planning_component("arm")
        self.arduinobot_gripper = self.arduinobot.get_planning_component("gripper")

        self.arm_state = RobotState(self.arduinobot.get_robot_model())
        self.gripper_state = RobotState(self.arduinobot.get_robot_model())


    def goalCallback(self, goal_handle):
        self.get_logger().info("Received goal request.")
        task_number = goal_handle.request.task_number

        if task_number == 1:
            arm_positions = np.array([1.57, 0.0, 0.0])
            gripper_positions = np.array([-0.7, -0.7])
        elif task_number == 2:
            arm_positions = np.array([0.0, -1.57, 0.0])
            gripper_positions = np.array([-0.5, -0.5])
        else:
            arm_positions = np.array([0.0, 0.0, 0.0])
            gripper_positions = np.array([0.0, 0.0])

        self.arm_state.set_joint_group_positions(arm_positions)
        self.gripper_state.set_joint_group_positions(gripper_positions)

        self.arduinobot_arm.set_start_state_to_current_state()
        self.arduinobot_gripper.set_start_state_to_current_state()

        self.arduinobot_arm.set_goal_state(robot_state=self.arm_state)
        self.arduinobot_gripper.set_goal_state(robot_state=self.gripper_state)

        arm_plan_result = self.arduinobot_arm.plan()
        gripper_plan_result = self.arduinobot_gripper.plan()

        if arm_plan_result and gripper_plan_result:
            self.arduinobot.execute(arm_plan_result.trajectory, controllers=[])
            self.arduinobot.execute(gripper_plan_result.trajectory, controllers=[])
        else:
            self.get_logger().error("Planning failed.")


def main():
    rclpy.init()
    task_server = TaskServer()
    rclpy.spin(task_server)
    task_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
