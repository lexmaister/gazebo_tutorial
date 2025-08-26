# Gazebo Tutorial: Controlling a Panda Arm in Gazebo Sim with Position Control (pos2_control)

## Introduction

This example package demonstrates how to control a Panda manipulator in Gazebo Sim using ROS2 position control (pos2_control), with topic bridging powered by ros_gz_bridge. TODO: extend

---

DRAFT:

create tutorial ws

```sh
mkdir -p tutorial_ws/src
cd tutorial_ws/src
```

download pkg code

* Go to: [GitHub Download Directory](https://download-directory.github.io/)
* Paste pkg address `https://github.com/lexmaister/gazebo_tutorial/tree/main/05_ros2_control/panda_gz_ros2_ctrl` and click `Download`

Unpack zip to `tutorial_ws/src` (example):

```sh
unzip ~/Downloads/lexmaister\ gazebo_tutorial\ main\ 05_ros2_control-panda_gz_ros2_ctrl.zip -d panda_gz_ros2_ctrl
```

Source ROS2 and build the package (jazzy example)

```sh
cd ..
colcon build --symlink-install
```

Source the environment and run main launch file

```sh
source install/setup.bash
```

USAGE

* launch with exact logger level

```sh
ros2 launch panda_gz_ros2_ctrl main.launch.py logger_level:=debug
```

IN other terminal:

```sh
source /opt/ros/jazzy/setup.bash
```

* send command to open gripper

```sh
ros2 action send_goal /panda_hand_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{
  "trajectory": {
    "joint_names": ["panda_finger_joint1"],
    "points": [
      {
        "positions": [0.04],
        "time_from_start": {"sec": 2, "nanosec": 0}
      }
    ]
  }
}'
```

* send command to close gripper

```sh
# Close action
ros2 action send_goal /panda_hand_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{
  "trajectory": {
    "joint_names": ["panda_finger_joint1"],
    "points": [
      {
        "positions": [0.00],
        "time_from_start": {"sec": 2, "nanosec": 0}
      }
    ]
  }
}'
```

* move arm left

```sh
ros2 action send_goal /panda_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{
  "trajectory": {
    "joint_names": [
      "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
      "panda_joint5", "panda_joint6", "panda_joint7"
    ],
    "points": [
      {
        "positions": [0.5, 0.2, 0.0, -1.2, 0.0, 1.4, 0.0],
        "time_from_start": {"sec": 4, "nanosec": 0}
      }
    ]
  }
}'
```

* move arm to park [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

```sh
ros2 action send_goal /panda_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{
  "trajectory": {
    "joint_names": [
      "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
      "panda_joint5", "panda_joint6", "panda_joint7"
    ],
    "points": [
      {
        "positions": [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
        "time_from_start": {"sec": 4, "nanosec": 0}
      }
    ]
  }
}'
```

* check controller data:

```sh
ros2 topic echo /joint_states sensor_msgs/msg/JointState
```

example message:

```text
header:
  stamp:
    sec: 118
    nanosec: 438000000
  frame_id: ''
name:
- panda_finger_joint1
- panda_finger_joint2
- panda_joint1
- panda_joint2
- panda_joint3
- panda_joint4
- panda_joint5
- panda_joint6
- panda_joint7
position:
- -7.858032258743763e-13
- -4.3655644420617645e-14
- 3.345305401580845e-07
- -0.7849834561347961
- 5.4490010370500386e-06
- -2.355966806411743
- -7.3978371801786125e-06
- 1.570957899093628
- 0.7849934697151184
velocity:
- 0.0
- 0.0
- 0.0
- 2.9793009161949158e-05
- 1.8917489796876907e-10
- 0.00011914223432540894
- 0.0
- -4.76837158203125e-05
- -2.384185791015625e-05
effort:
- 0.054768227306226436
- -1.7486450815340504e-05
- -0.17536401748657227
- -3.3343059420585632
- -6.135307312011719
- -15.096150875091553
- 0.21683472394943237
- 2.213101863861084
- 0.015369687242269947
---
```

* run demo sequence in another terminal

```sh
cd tutorial_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run panda_gz_ros2_ctrl demo_control
```
