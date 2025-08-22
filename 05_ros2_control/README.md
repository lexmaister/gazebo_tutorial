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