#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
import rclcpp
import time
from rclpy.logging import get_logger

rclcpp.init()
node = rclcpp.Node("mtc_node")

logger = get_logger("mtc_node")


def main():
    # Create a task
    task = core.Task()
    task.name = "current state"
    task.loadRobotModel(node)

    # Get the current robot state
    currentState = stages.CurrentState("current state")
    logger.info(f"Current stage properties:")
    for k, p in currentState.properties.items():
        logger.info(f"Property key={k}")
        logger.info(f"  name: {p.description()}")
        # logger.info(f"  type: {p.type()}")  # usually a string describing C++ type
        # logger.info(f"  doc:  {p.doc()}")

    logger.info("-------")
    logger.info(f"{currentState.properties.keys()}")
    logger.info("-------")

    # Add the stage to the task hierarchy
    task.add(currentState)

    # if task.plan():
    #     task.publish(task.solutions[0])

    # if task.plan():
    #     sol = task.solutions[0]
    #     for sub in sol.sub_trajectory():
    #         stage = sub.creator()  # this is a valid Stage view from C++
    #         logger.info(f"Stage in solution: {stage.name()}")

    #     # for k, v in currentState.properties.items():
    #     #     logger.info(f"solution start {k}: {v.value}")

    #     # task.introspection
    # else:
    #     print("!!! something went wrong !!!")
    time.sleep(1)
