# Gazebo Tutorial: Estimation of Joint Torque with Force Torque Sensor

## Introduction

This tutorial demonstrates how to set up, simulate, and validate a simple robotic arm with a force/torque sensor in **Gazebo**. You will measure the joint moment required to hold a load, control joint position, and compare simulated results with theoretical calculations.
A minimal setup is used: a single-link robot arm lifting a 1 kg ball. All simulation steps use the **SDF format** and leverage standard Gazebo sensors and plugins.

## Prerequisites

- **Gazebo (gz sim) installed** ([see project setup instructions](../README.md#references))
- SDF robot and world files exported (see previous tutorials or Blender/Phobos exports)
- Familiarity with basic Gazebo concepts and plugin configuration

## What You'll Learn

- How to add a static load to a robot model in SDF
- How to integrate and configure a force/torque sensor for a joint
- How to set up joint position control and state publishing plugins
- How to run the simulation and monitor sensor output topics
- How to validate simulation data against theoretical calculations

---

## 1. Robot Model Setup

- Use a robot arm model with a single movable link and a fixed 1 kg load (visualized as a green ball).
- **Export the robot from Blender (with Phobos)** and optimize the model for simulation by combining arm and load into a single link.
- Gazebo will merge links and recalculate center-of-mass, inertia, and visuals.
- For this tutorial, only one collision element is used (for the arm).

**Reference:** [Exporting Models from Blender with Phobos](../02.model_export_blender/README.md)

---

## 2. SDF File Editing

- Manually edit your exported `.sdf` model as follows:
  - In the `<joint>` section, define rotation axis and **remove unnecessary dynamic parameters** (we focus on static torque).
  - Add a **force_torque sensor** to the joint:
    - Typical config:
  
    ```xml
    <sensor name="arm_joint_ft" type="force_torque">
      <update_rate>2</update_rate>
      <topic>/arm_base_joint_ft</topic>
      <frame>moving_link</frame>
      <axis>x</axis>
    </sensor>
    ```

  - Reference the official [SDF Sensor Spec](https://sdformat.org/spec?ver=1.7&elem=sensor) for more details.

---

## 3. Controller Plugins

- Enable **JointPositionController** in the robot model for interactive angle control (PID tuned for smooth motion).
- Add **JointStatePublisher** for current joint state output.
- Example additions to SDF:

    ```xml
    <plugin name="arm_position_controller" filename="libJointPositionController.so">
      <!-- PID and joint config here -->
    </plugin>
    <plugin name="joint_state_publisher" filename="libJointStatePublisher.so">
      <!-- Joint state output config -->
    </plugin>
    ```

---

## 4. World File and Simulation Setup

- In your main world `.sdf`:
  - Include a `<gui>` section for manual joint control with GUI sliders.
  - Reference your robot model and configured plugins.

---

## 5. Running the Simulation

1. Place all SDF files in a dedicated directory (e.g., `~/gazebo_torque_sdf/`).
2. Launch Gazebo:

    ```sh
    gz sim torque_world.sdf
    ```

3. In the element tree, select your robot and use the GUI slider to adjust the arm's elevation.
4. Use terminal commands or Gazebo's GUI tools to monitor topics:
    - `/arm_base_joint_ft` — force/torque sensor output (focus on moment about X)
    - `/arm_base_joint_state` — joint position tracking

---

## 6. Analysis and Validation

- For static torque measurement:
  - Theoretical formula:  
      `Torque = total_mass × gravity × distance_to_COM × sin(angle)`
  - At key angles (e.g., 0.1, 0.3, 0.9, 1.3, 1.57 radians), compare the sensor's measured value with your calculation.
  - Results should closely align (verify minimal error/oscillation).
- This workflow validates both the **sensor model** and **controller performance** in Gazebo.

---

## 7. Video

[![video_tutorial_YT](https://img.youtube.com/vi/7ewrPQ2ovFk/0.jpg)](https://www.youtube.com/watch?v=7ewrPQ2ovFk)

---

## 8. References

- [Phobos repository](https://github.com/dfki-ric/phobos)
- [Sensors description in SDF](http://sdformat.org/spec?ver=1.6&elem=sensor)
- [AI & Robotics Lab Telegram Channel](https://t.me/ai_robotics_lab)
