#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory


class WaitForHandController(Node):
    """
    Helper node that blocks until the Panda hand trajectory controller
    action server is available, then exits with code 0.
    """

    def __init__(self):
        super().__init__("wait_for_hand_controller")

        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            "/panda_hand_controller/follow_joint_trajectory",
        )

        self.get_logger().info(
            "Waiting for /panda_hand_controller/follow_joint_trajectory action server..."
        )

        self._timer = self.create_timer(2.0, self._check_ready)

    def _check_ready(self):
        # Try to connect with a short timeout
        if self._client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info(
                "Panda hand controller action server is available. Exiting helper node."
            )
            # Shut down cleanly; the process will exit
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaitForHandController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure we shut down if not already done in _check_ready()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
