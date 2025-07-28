# Gazebo Tutorial: Export From Blender With Phobos - Basic Elements

This tutorial explains how to prepare a 3D model in Blender, define its kinematic and dynamic properties using the Phobos plugin, and export it as an SDF file for simulation in Gazebo.

## Key steps
- **Model the robot** in Blender, with separate objects for each link (`base` and `arm`).
- **Use the Phobos plugin** to define the robot's structure:
  - Create `links` for the base and the arm.
  - Define `visual`, `collision`, and `inertial` properties for each link.
  - Create a `revolute` joint to connect the two links.
- **Export the model** from Phobos in `urdf` format.
- **Convert the `urdf` file to `sdf`** using Gazebo's command-line tools.

## How to run
Navigate to the `robot_model` -> `sdf` directory and run simulation:
```bash
cd robot_model/sdf
gz sim main.sdf
```

## Video
[![video_tutorial_YT](https://img.youtube.com/vi/3DswcUj0HLc/0.jpg)](https://www.youtube.com/watch?v=3DswcUj0HLc)

## References
- [Phobos repository](https://github.com/dfki-ric/phobos)
- [Gazebo SDF Documentation](https://gazebosim.org/docs/all/sdf)
- [Model kinematics description in SDF](http://sdformat.org/tutorials?tut=spec_model_kinematics&cat=specification&)