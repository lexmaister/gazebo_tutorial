# Gazebo Tutorial: Connecting ROS 2 and Gazebo Simulation with ros_gz_bridge

## Introduction

This tutorial demonstrates how to connect and control a Panda manipulator in **Gazebo** using **ROS 2** and the [`ros_gz_bridge`](https://github.com/gazebosim/ros_gz) package. You’ll learn how to set up model plugins for joint command and state, bridge topics to ROS 2, and use RQT tools for interactive robot control and monitoring. The materials are designed for quick integration and minimal manual edits, following practices from the advanced Gazebo tutorial collection.

---

## Prerequisites

- **Gazebo Harmonic** or later (`gz sim` installed)  
  ([see project setup instructions](../README.md#references))
- **ROS 2 Jazzy** or compatible distribution; sourced and functional
- [`ros_gz_bridge` package](https://github.com/gazebosim/ros_gz) installed
- Local copy of the Panda model at  
  `~/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/panda`
- Familiarity with ROS 2 topics, Gazebo SDF editing, and RQT GUI

---

## What You'll Learn

- Launching a Panda manipulator in local Gazebo simulation using SDF files
- Editing SDF to integrate plugins for joint command and state publishing
- Setting up topic bridges with ros_gz_bridge between Gazebo and ROS 2
- Using scripts and RQT for live robot control and state monitoring

---

## 1. Panda Model Setup

- The simulation uses the standard Panda manipulator.
- You can use your own model, but for this tutorial, you should use the one already present at  
  `~/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/panda`
- All required SDF and plugin configurations are prepared in the repository [materials](https://github.com/lexmaister/gazebo_tutorial/tree/main/04_ros2_communication).

---

## 2. SDF File Editing

You can **either use the provided `model.sdf` from this repository** (recommended for a quick start), or manually update your local model file to enable ROS 2–Gazebo communication.

### Option 1: Use `model.sdf` from the Repository**

- Copy the supplied `model.sdf` to your local Panda model folder.
- This file includes all necessary plugins with correct topic names for direct bridging.

### Option 2: Manually Update Your Local Model**

- Open your local file:
  
```sh
~/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/panda/model.sdf
```

- Edit the `<model>` section. Add a controller and state publisher plugin for each joint you wish to control or observe, for example:

  ```xml
  <plugin filename="ignition-gazebo-joint-position-controller-system" name="ignition::gazebo::systems::JointPositionController">
      <joint_name>panda_joint1</joint_name>
      <p_gain>50</p_gain>
      <d_gain>20</d_gain>
      <i_gain>0</i_gain>
      <i_min>-1</i_min>
      <i_max>1</i_max>
      <cmd_min>-87</cmd_min>
      <cmd_max>87</cmd_max>
      <topic>j1_ctrl</topic>
  </plugin>

  <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <joint_name>panda_joint1</joint_name>
      <topic>/j1_state</topic>
  </plugin>
  ```

- **Repeat** for every joint you want to interface.
- Adjust PID values for your needs.
- For reference on SDF plugin syntax, see the [official SDF documentation](https://sdformat.org/spec?ver=1.7&elem=plugin).

---

## 3. Controller Plugins

- The setup uses topic-based controllers and state publishers for each joint, using your chosen topic names (such as `j1_ctrl`, `/j1_state`, etc.).
- These plugin blocks are included in the `model.sdf` (see above) and form the basis for topic bridging.
- Use the provided scripts and SDF examples for fastest integration.

---

## 4. World File and Simulation Setup

No manual changes to the world file are required for this tutorial.  
All model and plugin configuration is handled via the SDF file and provided scripts.  
You can launch the simulation directly.

---

## 5. Running the Simulation

1. Place all provided files (model, plugins, scripts) in your working directory.
2. Start Gazebo simply by running:

  ```sh
  gz sim
  ```

3. The default Gazebo world will load your configured Panda manipulator from the local model path if your SDF edits are correct.

---

## 6. ROS 2 Bridging and RQT Launch

1. Open a new terminal and change to your working directory.
2. Make the bridge and RQT script executable:

  ```sh
  chmod +x run_bridge_and_rqt.sh
  ```

3. Launch the script to start all bridging processes and open the RQT GUI:

  ```sh
  ./run_bridge_and_rqt.sh
  ```

This script starts all required `ros_gz_bridge` processes based on your SDF plugin and topic configuration and loads the RQT GUI for interactive control and monitoring.

---

## 7. RQT GUI Usage

- With the simulation and bridge running, use the supplied `panda.perspective` file to instantly load a pre-configured RQT workspace.
- In RQT, you can:
  - Send commands to joint position topics for the Panda manipulator
  - View feedback from joint states in real time
  - Plot and analyze joint and controller behavior as you experiment

---

## 8. Video

<!-- [![video_tutorial_YT](https://img.youtube.com/vi/7ewrPQ2ovFk/0.jpg)](https://www.youtube.com/watch?v=7ewrPQ2ovFk) -->

---

## 9. References

- [`ros_gz_bridge` package](https://github.com/gazebosim/ros_gz)
<!-- - [04_ros2_communication: Tutorial Materials](https://github.com/lexmaister/gazebo_tutorial/tree/main/04_ros2_communication) -->
- [Gazebo Fuel Panda Model](https://app.gazebosim.org/OpenRobotics/fuel/models/Panda%20with%20Ignition%20position%20controller%20model)
- [SDF Plugin Documentation](https://sdformat.org/spec?ver=1.7&elem=plugin)
- [General Gazebo Tutorials Collection](https://github.com/lexmaister/gazebo_tutorial)

---

**Tip:**  
Fine-tune PID values as needed and ensure your bridging topic names match between plugins and the ROS 2 side. Advanced plotting can be done in RQT with pyqtgraph or matplotlib plugins.
