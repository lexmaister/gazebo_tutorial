# Gazebo Tutorial: Planning Panda Arm's motion in MoveIt

* extending functionality of `panda_gz_ros2_ctrl`
* using `moveit_resources_panda_description` pkg for a panda robot description: geometry, inertia
* construct higher level XACRO:
  * place fobot in the `world` frame and fix it
  * add ros2_cotrol elements
  * add ros_gz_control element to bridge between Gazebo and ros2_ctrl
* check robot position after launch, in another terminal with ros activated:

```bash
ros2 topic echo /tf_static
```

should contain:

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

* Add MoveIt elements to package got with MoveIt Assistant:
  * inside `config`:
    * `moveit.rviz`
    * `moveit_controllers.yaml`
    * `kinematics.yaml`
    * `pilz_cartesian_limits.yaml`
    * `joint_limits.yaml` with enabled acceleration limits
      * set `has_acceleration_limits` to `true`
      * set acceleration limits: for arm joints to 5.0, for fingers to 0.01
      * set fingers velosity limit 0.01
  * to `srdf`:
    * `panda.srdf`



* URDF: `moveit_ws/src/moveit_resources/panda_description/urdf/panda.urdf.xacro`
* update `config/joint_limits.yaml`:
  * set `has_acceleration_limits` to `true`
  * set acceleration limits: for arm joints to 5.0, for fingers to 0.01
  * set fingers velosity limit 0.01
* unset `MotionPlanning` -> `Planned Path` -> `Loop Animation`
* create `gazebo.launch.py`
* update `config/panda.ros2_control.xacro`:
  
```xml
...
<ros2_control name="${name}" type="system">
    <hardware>
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    ...
```

* update `config/panda.urdf.xacro`:
  
```xml
<xacro:panda_ros2_control name="PandaGazebo" ... />
```

* uncomment and correct in package.xml

```xml
<exec_depend>joint_trajectory_controller</exec_depend>
<exec_depend>gz_ros2_control</exec_depend>
```


## Video

<!-- [![video_tutorial_YT](https://img.youtube.com/vi/LciIEdbv88U/0.jpg)](https://www.youtube.com/watch?v=LciIEdbv88U) -->

## References

[MoveIt Setup Assistant Example](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

<!-- - [`ros2_control` package documentation (jazzy)](https://control.ros.org/jazzy/doc/ros2_control/doc/index.html)
- [`gz_ros2_control` package documentation (jazzy)](https://github.com/ros-controls/gz_ros2_control/blob/jazzy/doc/index.rst)
- [General Gazebo Tutorials Collection](https://github.com/lexmaister/gazebo_tutorial) -->
