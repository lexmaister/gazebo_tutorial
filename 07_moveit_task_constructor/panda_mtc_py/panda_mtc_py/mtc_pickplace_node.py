#! /usr/bin/env python3

import time
import math

import rclcpp
from rclpy.logging import get_logger

from moveit.task_constructor import core, stages
from moveit_msgs.msg import Constraints, OrientationConstraint

from geometry_msgs.msg import TwistStamped, PoseStamped, Pose, Quaternion, Point
from std_msgs.msg import Header


rclcpp.init()
node = rclcpp.Node("mtc_node")
logger = get_logger("mtc_node")


def main():
    """
    Simple MTC pick and place task using both 'arm' and 'hand' planning group:
    based on tutorial: https://github.com/moveit/moveit_task_constructor/blob/jazzy/demo/scripts/pickplace.py
    """

    arm = "panda_arm"
    hand = "hand"

    # Create a task
    task = core.Task()
    task.name = "pick_place"
    task.loadRobotModel(node)

    object_name = "target"

    # frames
    world_header = Header(frame_id="world")
    panda_hand_header = Header(frame_id="panda_hand")

    # Planner instance that is used in Connect stages for arm
    pipeline_planner = core.PipelinePlanner(node, "ompl", "RRTConnectkConfigDefault")
    planners = [(arm, pipeline_planner)]

    # Slow gripper planner
    slow_gripper = core.JointInterpolationPlanner()
    slow_gripper.max_velocity_scaling_factor = 0.05

    # ------------------------------------------------------------------
    # 1) Current state
    # ------------------------------------------------------------------
    current_state = stages.CurrentState("Current state")
    task.add(current_state)

    # ------------------------------------------------------------------
    # 2) Open hand - to fit with pregrasp condition
    # ------------------------------------------------------------------
    open_hand = stages.MoveTo("Open hand", slow_gripper)
    open_hand.group = hand
    open_hand.setGoal("open")
    task.add(open_hand)

    # ------------------------------------------------------------------
    # 3) Connect the current robot state with the solutions of the following Pick stage
    # ------------------------------------------------------------------
    task.add(stages.Connect("Connect current - pick", planners))

    # ------------------------------------------------------------------
    # 4) Pick: grasp 'target' object
    # https://github.com/moveit/moveit_task_constructor/blob/jazzy/core/include/moveit/task_constructor/stages/pick.h
    # https://github.com/moveit/moveit_task_constructor/blob/jazzy/core/src/stages/simple_grasp.cpp
    # ------------------------------------------------------------------
    # The grasp generator spawns a set of possible grasp poses around the object
    grasp_generator = stages.GenerateGraspPose("Generate Grasp Pose")
    grasp_generator.angle_delta = math.pi / 2
    grasp_generator.pregrasp = "open"
    grasp_generator.grasp = "close"
    # Generate solutions for all initial states
    grasp_generator.setMonitoredStage(task["Current state"])

    # SimpleGrasp container encapsulates IK calculation of arm pose as well as finger closing
    simpleGrasp = stages.SimpleGrasp(grasp_generator, slow_gripper, "Grasp")

    # Goal pose for the panda_hand frame
    ik_frame = PoseStamped(
        header=panda_hand_header,
        pose=Pose(
            position=Point(x=0.0, y=0.0, z=0.1034),  # in the center between the fingers
            orientation=Quaternion(
                x=0.0, y=0.7071, z=0.0, w=0.7071
            ),  # from the cartesian example - finish panda_hand frame pose
        ),
    )
    simpleGrasp.setIKFrame(ik_frame)

    # Pick container comprises approaching, grasping (using SimpleGrasp stage), and lifting of object
    pick = stages.Pick(simpleGrasp, "Pick")
    pick.eef = hand
    pick.object = object_name

    # Twist to approach the object - from side
    approach = TwistStamped(header=world_header)
    approach.twist.linear.x = 0.3
    pick.setApproachMotion(motion=approach, min_distance=0.03, max_distance=0.1)

    # Twist to lift the object
    lift = TwistStamped(header=world_header)
    lift.twist.linear.z = 1.0
    pick.setLiftMotion(motion=lift, min_distance=0.03, max_distance=0.1)

    # Add the pick stage to the task's stage hierarchy
    task.add(pick)

    # ------------------------------------------------------------------
    # 5) Connect the Pick stage with the following Place stage
    # https://github.com/moveit/moveit_task_constructor/blob/jazzy/core/include/moveit/task_constructor/stages/connect.h
    # ------------------------------------------------------------------
    oc = OrientationConstraint()
    oc.parameterization = oc.ROTATION_VECTOR
    oc.header = world_header
    oc.link_name = object_name
    oc.orientation.w = 1.0
    oc.absolute_x_axis_tolerance = math.pi / 3  # 60°
    oc.absolute_y_axis_tolerance = math.pi / 3
    oc.absolute_z_axis_tolerance = math.pi
    oc.weight = 1.0

    constraints = Constraints()
    constraints.name = "target:upright"
    constraints.orientation_constraints.append(oc)

    con = stages.Connect("Connect pick - place", planners)
    con.path_constraints = constraints
    con.timeout = 7.0  # increase due to constraints calc
    task.add(con)

    # ------------------------------------------------------------------
    # 6) Plase the 'target' object
    # ------------------------------------------------------------------
    # Pose that the object should have after placing
    # ! use this format of pose definition
    finish_pose = PoseStamped()
    finish_pose.header = world_header
    finish_pose.pose.position.x = 0.45
    finish_pose.pose.position.y = -0.5
    finish_pose.pose.position.z = 0.45
    finish_pose.pose.orientation.w = 1.0

    place_generator = stages.GeneratePlacePose("Generate Place Pose")
    place_generator.setMonitoredStage(task["Pick"])
    place_generator.object = object_name
    place_generator.pose = finish_pose

    simpleUnGrasp = stages.SimpleUnGrasp(place_generator, slow_gripper, "UnGrasp")

    place = stages.Place(simpleUnGrasp, "Place")
    place.eef = hand
    place.object = object_name
    place.eef_frame = "panda_link8"

    # Twist to retract from the object
    retract = TwistStamped(header=world_header)
    retract.twist.linear.z = 1.0
    place.setRetractMotion(retract, 0.03, 0.1)

    # Twist to place the object
    placeMotion = TwistStamped(header=panda_hand_header)
    placeMotion.twist.linear.x = -0.5
    place.setPlaceMotion(placeMotion, 0.03, 0.1)

    # Add the place pipeline to the task's hierarchy
    task.add(place)

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
