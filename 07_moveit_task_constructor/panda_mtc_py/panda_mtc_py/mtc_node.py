#! /usr/bin/env python3

import time
import rclcpp
from moveit.task_constructor import core, stages
from geometry_msgs.msg import TwistStamped, Twist, Vector3Stamped, Vector3
from std_msgs.msg import Header
from rclpy.logging import get_logger


rclcpp.init()
node = rclcpp.Node("mtc_node")

logger = get_logger("mtc_node")


def main():
    # Specify the planning group
    group = "panda_arm"

    # Create a task
    task = core.Task()
    task.name = "cartesian"
    task.loadRobotModel(node)

    # Get the current robot state
    currentState = stages.CurrentState("current state")
    # Add the stage to the task hierarchy
    task.add(currentState)

    # Create a planner instance that is used to connect
    # the current state to the grasp approach pose
    pipelinePlanner = core.PipelinePlanner(node, "ompl", "RRTConnectkConfigDefault")
    planners = [(group, pipelinePlanner)]

    # move along x
    cartesian = core.CartesianPath()
    move = stages.MoveRelative("x +0.2", cartesian)
    move.group = group
    header = Header(frame_id="world")
    move.setDirection(
        Vector3Stamped(header=header, vector=Vector3(x=0.2, y=0.0, z=0.0))
    )
    task.add(move)

    if task.plan():
        task.publish(task.solutions[0])
        logger.info("PLANNING RESULTS ARE PUBLISHED")
        # Keep node alive to inspect in RViz
        time.sleep(3600)
    else:
        logger.error("CAN'T PLAN TASK!")

    # avoid ClassLoader warning
    del planners

    logger.info(f"MTC node is finished")
