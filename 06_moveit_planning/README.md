# Gazebo Tutorial: Planning Panda Arm's motion in MoveIt

## Introduction

This tutorial demonstrates how to add motion planning capabilities to the Panda robot simulation using MoveIt 2. It builds directly on the setup from the [Gazebo Tutorial: Controlling a Panda Arm in Gazebo with ros2_control](https://github.com/lexmaister/gazebo_tutorial/tree/main/05_ros2_control/), extending its functionality to allow for complex, collision-aware path planning.

In this guide, you will learn how to:

* Integrate the MoveIt 2 framework into an existing ros2_control-based robot package.
* Configure the robot's semantic description (SRDF) and controller settings for MoveIt.
* Use a higher-level XACRO file to correctly position the robot in the simulation world.
* Launch Gazebo, RViz, and all necessary MoveIt nodes to plan and execute motion paths.
* Introduce objects into the planning scene for collision avoidance.

## Prerequisites

The MoveIt 2 packages are installed for your ROS 2 distribution (e.g., Jazzy):

```bash
sudo apt update && sudo apt install ros-jazzy-moveit
```

It's recommended to complete the previous tutorial: [Controlling a Panda Arm in Gazebo with ros2_control](ttps://github.com/lexmaister/gazebo_tutorial/tree/main/05_ros2_control/) and also [MoveIt Setup Assistant Example](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html).

## Integrating MoveIt with the Panda Robot

### Higher-Level XACRO for World Placement

To properly integrate with MoveIt, which requires a fixed world frame, we construct a higher-level XACRO file `urdf/panda.urdf.xacro`. This file will:

* Reference the existing Panda robot description from the `moveit_resources_panda_description` package.
* Create a fixed joint between a `world` frame and the robot's base link `panda_link0`.
* Include the necessary `ros2_control` and `ros_gz_control` plugin definitions to bridge Gazebo with ros2_control.

### Verifying the Robot's Position

Once the simulation is launched, it's crucial to verify that the robot is correctly positioned in the world frame. Open a new terminal (with the ROS 2 environment sourced) and run:

```bash
ros2 topic echo /tf_static
```

The output should confirm that `panda_link0` is correctly attached to the `world` frame, with the specified translation and rotation:

```bash
transforms:
- header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: world
  child_frame_id: panda_link0
  transform:
    translation:
      x: -0.1
      y: -0.1
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
...
```

### MoveIt Configuration Files

We use the MoveIt Setup Assistant to generate the initial MoveIt configuration package. The key files are then customized as follows:

* `srdf/panda.srdf`: The Semantic Robot Description File (SRDF) complements the URDF. It defines joint groups (like panda_arm and hand), end-effectors, and default robot states (e.g., a "ready" pose).
* `config/moveit_controllers.yaml`: Specifies the controller types and action namespaces used by MoveIt to command the robot.
* `config/joint_limits.yaml`: Defines velocity and acceleration limits for each joint. It is critical to enable acceleration limits for smoother motion:
  * Set has_acceleration_limits to true.
  * Define acceleration limits: 5.0 for arm joints and 0.01 for finger joints.
  * Set a velocity limit for the finger joints to 0.01.

Other Files:

* `config/kinematics.yaml`: Configures the kinematics solver.
* `config/pilz_cartesian_limits.yaml`: Defines limits for Cartesian path planning.
* `config/moveit.rviz`: An RViz configuration file pre-loaded with the MotionPlanning plugin for interactive planning.

## Launching and Interacting with the Simulation

### Main Launch and Scene Objects

The project contains multiple launch files:

* `main.launch.py`: The primary launch file that starts Gazebo, RViz, the robot state publisher, and all required MoveIt 2 nodes (e.g., move_group).
* `spawn_wall.launch.py`: An additional launch file that uses the information from `sdf/wall.sdf` to spawn a wall into the Gazebo to simulate obstacle.

There is also `config/wall.scene` to import the same wall into the MoveIt planning scene, allowing for collision-aware motion planning.

### Interacting with RViz

In the RViz window, under the `MotionPlanning` panel:

* Navigate to the `Planned Path` tab.
* Uncheck the `Loop Animation` option to prevent the planned path from repeating endlessly.
* Use the interactive markers to drag the end-effector to a desired goal state and click the `Plan & Execute` button to see the robot move in both RViz and Gazebo.

## Video

<!-- [![video_tutorial_YT](https://img.youtube.com/vi/LciIEdbv88U/0.jpg)](https://www.youtube.com/watch?v=LciIEdbv88U) -->

## References

* [MoveIt 2 Documentetion](https://moveit.picknik.ai/main/index.html)
* [ROS + Gazebo Sim](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/README.md)
* [`ros2_control` package documentation (jazzy)](https://control.ros.org/jazzy/doc/ros2_control/doc/index.html)
* [`gz_ros2_control` package documentation (jazzy)](https://github.com/ros-controls/gz_ros2_control/blob/jazzy/doc/index.rst)
* [General Gazebo Tutorials Collection](https://github.com/lexmaister/gazebo_tutorial)

## Similar projects

* [panda_gz_moveit2](https://github.com/AndrejOrsula/panda_gz_moveit2/tree/jazzy)
* [how to control a robotic arm using ros 2 control and gazebo](https://automaticaddison.com/how-to-control-a-robotic-arm-using-ros-2-control-and-gazebo/)
