import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class DemoControllerNode(Node):
    def __init__(self):
        super().__init__('demo_controller_node')
        self.get_logger().info('Starting demo controller node...')

        self._arm_action_client = ActionClient(self, FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory')
        self._hand_action_client = ActionClient(self, FollowJointTrajectory, '/panda_hand_controller/follow_joint_trajectory')

        # Define joint poses
        self.park_pose = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        self.reach_pose = [0.5, 0.2, 0.0, -1.2, 0.0, 1.4, 0.0]
        
        # Gripper positions
        self.gripper_open = [0.04]
        self.gripper_closed = [0.0]

        self.get_logger().info('Waiting for action servers...')
        self._arm_action_client.wait_for_server()
        self._hand_action_client.wait_for_server()
        self.get_logger().info('Action servers available.')

        # Timer to start the demo sequence after a short delay
        self.timer = self.create_timer(1.0, self.start_demo_sequence)

    def start_demo_sequence(self):
        """Starts the first step of the robot motion sequence."""
        self.timer.cancel()  # Ensure it runs only once
        self.get_logger().info('Starting robot motion sequence...')
        
        # Start the chain of actions by moving to the park pose first.
        # The callback 'self.move_to_reach_pose' will be triggered upon completion.
        self.send_arm_goal(self.park_pose, on_done_callback=self.move_to_reach_pose)

    def move_to_reach_pose(self, future):
        """Callback executed after reaching the park pose. Moves to the reach pose."""
        self.get_logger().info('Moving to reach pose.')
        self.send_arm_goal(self.reach_pose, on_done_callback=self.open_gripper)
    
    def open_gripper(self, future):
        """Callback executed after reaching the reach pose. Opens the gripper."""
        self.get_logger().info('Opening gripper.')
        self.send_hand_goal(self.gripper_open, on_done_callback=self.close_gripper)

    def close_gripper(self, future):
        """Callback executed after opening the gripper. Closes the gripper."""
        self.get_logger().info('Closing gripper.')
        self.send_hand_goal(self.gripper_closed, on_done_callback=self.return_to_park)

    def return_to_park(self, future):
        """Callback executed after closing the gripper. Returns to park pose."""
        self.get_logger().info('Returning to park pose.')
        self.send_arm_goal(self.park_pose, on_done_callback=self.finish_demo)

    def finish_demo(self, future):
        """Callback executed after returning to park. Shuts down the node."""
        self.get_logger().info('Demo sequence finished successfully.')
        rclpy.shutdown()

    def send_arm_goal(self, joint_positions, on_done_callback=None, duration_sec=4.0):
        """Sends a goal to the arm controller."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [f'panda_joint{i+1}' for i in range(7)]
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in joint_positions]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        goal_msg.trajectory.points.append(point)

        send_goal_future = self._arm_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, on_done_callback))

    def send_hand_goal(self, joint_positions, on_done_callback=None, duration_sec=2.0):
        """Sends a goal to the hand controller."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['panda_finger_joint1']
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in joint_positions]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        goal_msg.trajectory.points.append(point)

        send_goal_future = self._hand_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, on_done_callback))

    def goal_response_callback(self, future, on_done_callback):
        """Generic callback for goal acceptance."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted.')
        get_result_future = goal_handle.get_result_async()
        
        # Attach the specific 'on_done' callback for the result
        if on_done_callback:
            get_result_future.add_done_callback(on_done_callback)


def main(args=None):
    rclpy.init(args=args)
    action_client_node = DemoControllerNode()
    rclpy.spin(action_client_node)
    action_client_node.destroy_node()


if __name__ == '__main__':
    main()