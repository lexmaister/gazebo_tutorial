#! /usr/bin/env python3

import time
import math

import rclcpp
from rclpy.logging import get_logger

from moveit.task_constructor import core, stages

from geometry_msgs.msg import (
    TwistStamped,
    PoseStamped,
)
from std_msgs.msg import Header


rclcpp.init()
node = rclcpp.Node("mtc_node")
logger = get_logger("mtc_node")


def main():
    """
    Simple MTC pick and place task using both 'arm' and 'hand' planning group:
    Task "pick_place"
    ├── Stage "current_state"
    ├── Stage "connect" (Connect)
    ├── Container "pick"
    │    ├── Generate grasp pose(s)
    │    ├── Move to pre-grasp
    │    ├── Close gripper / attach
    │    ├── Lift object
    └── Container "place"
        ├── Move to place pre-pose
        ├── Lower
        ├── Open gripper / detach
        └── Retreat
    based on tutorial: https://moveit.github.io/moveit_task_constructor/tutorials/pick-and-place.html
    """

    arm = "panda_arm"
    hand = "hand"

    # Create a task
    task = core.Task()
    task.name = "pick_place"
    task.loadRobotModel(node)

    object_name = "target"

    # frames
    world = "world"
    panda_hand = "panda_hand"

    # ------------------------------------------------------------------
    # 1) Current state
    # ------------------------------------------------------------------
    current_state = stages.CurrentState("Current state")
    task.add(current_state)

    # ------------------------------------------------------------------
    # 2) Open hand - to fit with pregrasp condition
    # ------------------------------------------------------------------
    open_hand = stages.MoveTo("open hand", core.JointInterpolationPlanner())
    open_hand.group = hand
    open_hand.setGoal("open")
    task.add(open_hand)

    # ------------------------------------------------------------------
    # 3) Connect the current robot state with the solutions of the following stages
    # ------------------------------------------------------------------
    # Create a planner instance that is used to connect
    # the current state to the grasp approach pose
    pipeline_planner = core.PipelinePlanner(node, "ompl", "RRTConnectkConfigDefault")
    planners = [(arm, pipeline_planner)]

    # Connect the current and pick stages
    task.add(stages.Connect("Connect", planners))

    # ------------------------------------------------------------------
    # 4) Pick: grasp 'target' object
    # https://github.com/moveit/moveit_task_constructor/blob/jazzy/core/include/moveit/task_constructor/stages/pick.h
    # ------------------------------------------------------------------
    # The grasp generator spawns a set of possible grasp poses around the object
    grasp_generator = stages.GenerateGraspPose("Generate Grasp Pose")
    grasp_generator.angle_delta = math.pi / 2
    grasp_generator.pregrasp = "open"
    grasp_generator.grasp = "close"
    # Generate solutions for all initial states
    grasp_generator.setMonitoredStage(task["Current state"])

    # SimpleGrasp container encapsulates IK calculation of arm pose as well as finger closing
    simpleGrasp = stages.SimpleGrasp(grasp_generator, "Grasp")

    # from the cartesian example - finish panda_hand frame pose
    ik_frame = PoseStamped()
    ik_frame.header.frame_id = panda_hand
    ik_frame.pose.position.z = 0.1034  # tcp between fingers
    # Set frame for IK calculation in the center between the fingers
    ik_frame.pose.orientation.x = 0.0
    ik_frame.pose.orientation.y = 0.7071
    ik_frame.pose.orientation.z = 0.0
    ik_frame.pose.orientation.w = 0.7071
    simpleGrasp.setIKFrame(ik_frame)

    # Pick container comprises approaching, grasping (using SimpleGrasp stage), and lifting of object
    pick = stages.Pick(simpleGrasp, "Pick")
    pick.eef = hand
    pick.object = object_name

    # Twist to approach the object
    approach = TwistStamped()
    approach.header.frame_id = world
    approach.twist.linear.x = 0.3
    pick.setApproachMotion(motion=approach, min_distance=0.03, max_distance=0.1)

    # Twist to lift the object
    lift = TwistStamped()
    lift.header.frame_id = world
    lift.twist.linear.z = 1.0
    pick.setLiftMotion(motion=lift, min_distance=0.03, max_distance=0.1)

    # Add the pick stage to the task's stage hierarchy
    task.add(pick)

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
