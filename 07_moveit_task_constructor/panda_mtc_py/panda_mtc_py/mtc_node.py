#! /usr/bin/env python3

import time
import math

import rclcpp
from rclpy.logging import get_logger

from moveit.task_constructor import core, stages

from geometry_msgs.msg import (
    Vector3Stamped,
    Vector3,
    PointStamped,
    Point,
    PoseStamped,
    Pose,
    Quaternion,
)
from std_msgs.msg import Header


rclcpp.init()
node = rclcpp.Node("mtc_node")
logger = get_logger("mtc_node")


def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    r = roll
    p = pitch
    y = yaw
    sin = math.sin
    cos = math.cos
    qx = sin(r / 2) * cos(p / 2) * cos(y / 2) - cos(r / 2) * sin(p / 2) * sin(y / 2)
    qy = cos(r / 2) * sin(p / 2) * cos(y / 2) + sin(r / 2) * cos(p / 2) * sin(y / 2)
    qz = cos(r / 2) * cos(p / 2) * sin(y / 2) - sin(r / 2) * sin(p / 2) * cos(y / 2)
    qw = cos(r / 2) * cos(p / 2) * cos(y / 2) + sin(r / 2) * sin(p / 2) * sin(y / 2)

    return [qx, qy, qz, qw]


def quaternion_multiply(q0, q1):
    """
    Multiplies two quaternions.

    Input
        :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
        :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

    Output
        :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)

    """
    # Extract the values from q0
    x0 = q0[0]
    y0 = q0[1]
    z0 = q0[2]
    w0 = q0[3]

    # Extract the values from q1
    x1 = q1[0]
    y1 = q1[1]
    z1 = q1[2]
    w1 = q1[3]

    # Compute the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = [q0q1_w, q0q1_x, q0q1_y, q0q1_z]

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion


def main():
    """
    Simple MTC cartesian using both 'arm' and 'hand' planning group.
    """

    arm = "panda_arm"
    hand = "hand"

    # Create a task
    task = core.Task()
    task.name = "cartesian"
    task.loadRobotModel(node)

    # Planner for arm motions
    pipeline_planner = core.PipelinePlanner(node, "ompl", "RRTConnectkConfigDefault")
    planners = [(arm, pipeline_planner)]

    cartesian = core.CartesianPath()
    joint_interp = core.JointInterpolationPlanner()
    world_header = Header(frame_id="world")

    # ------------------------------------------------------------------
    # 1) Current state
    # ------------------------------------------------------------------
    current_state = stages.CurrentState("current state")
    task.add(current_state)

    # ------------------------------------------------------------------
    # 2) Approach: move and rotate closer to the 'target' position
    # ------------------------------------------------------------------
    q1 = quaternion_from_euler(0, math.pi / 2, 0)
    q2 = quaternion_from_euler(0, 0, math.pi * 3 / 4)
    qx, qy, qz, qw = quaternion_multiply(q1, q2)

    approach = stages.MoveTo("approach target", cartesian)
    approach.group = arm
    approach.setGoal(
        PoseStamped(
            header=world_header,
            pose=Pose(
                position=Point(x=0.15, y=0.3, z=0.45),
                orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
            ),
        )
    )
    task.add(approach)

    # ------------------------------------------------------------------
    # 3) Open hand
    # ------------------------------------------------------------------

    open_hand = stages.MoveTo("open hand", joint_interp)
    open_hand.group = hand
    open_hand.setGoal("open")
    task.add(open_hand)

    # ------------------------------------------------------------------
    # 4) Final small move to the grasp position
    # ------------------------------------------------------------------
    grasp_move = stages.MoveRelative("move to target", cartesian)
    grasp_move.group = arm
    grasp_move.setDirection(
        Vector3Stamped(
            header=world_header,
            vector=Vector3(x=0.195, y=0.0, z=0.0),
        )
    )
    task.add(grasp_move)

    # ------------------------------------------------------------------
    # Plan and execute
    # ------------------------------------------------------------------
    if task.plan():
        logger.info(f"Found {len(task.solutions)} solution(s), publishing first one")
        task.publish(task.solutions[0])
        logger.info("PLANNING RESULTS ARE PUBLISHED")
        # time to observe and execute the results
        time.sleep(3600)
    else:
        logger.error("CAN'T PLAN TASK!")

    # avoid ClassLoader warning
    del pipeline_planner
    del planners
    logger.info("MTC node is finished")


if __name__ == "__main__":
    main()
