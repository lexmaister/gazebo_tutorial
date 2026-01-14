# Gazebo Tutorial: Planning Panda Arm's tasks in Moveit Task Constructor on Python

## Introduction

This tutorial shows how to plan and visualize a different tasks for the
Franka Emika Panda arm using **MoveIt Task Constructor (MTC) in Python**, with
the robot simulated in **Gazebo** and integrated through the `panda_gz_moveit`
environment.
In this example you will:

- Launch a ready‑to‑use Gazebo + MoveIt environment for the Panda arm.
- Load a composite obstacle and a thin cylindrical target into the MoveIt
  PlanningScene from YAML files.
- Run a Python MTC node that provides one of the example tasks:
  - cartesian movement,
  - pick and place target object.

## Prerequisites

It's recommended to complete the previous tutorial: [Planning Panda Arm's motion in MoveIt](https://github.com/lexmaister/gazebo_tutorial/tree/main/06_moveit_planning) that will be used as environment here and also learn [MoveIt Task Constructor concepts](https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor/moveit_task_constructor.html).

## Package Overview

This tutorial is implemented as a ROS 2 package named `panda_mtc_py`. It
contains:

- A **launch file** that orchestrates Gazebo + MoveIt + helper nodes:
  - `launch/cartesian.launch.py`
  - `launch/pickplace.launch.py`
- **Helper nodes**:
  - `wait_env_ready.py` – waits until the Panda hand controller is available.
  - `add_scene_from_yaml.py` – loads collision objects from YAML files into
    the MoveIt planning scene.
- **Main MTC node**:
  - `mtc_node.py` – defines a simple MTC Cartesian task.
  - `mtc_pickplace_node.py` – defines a simple MTC Pick and Place task.
- **Configuration files**:
  - `config/scene.yaml` – composite obstacle description.
  - `config/target.yaml` – target object description.
  - `config/rviz_mtc.rviz` – RViz configuration used by the launch file.

The `panda_mtc_py` package depends on the `panda_gz_moveit` package, which
provides the Panda robot model, Gazebo world, controllers and MoveIt setup.

## Setup and Installation

This project is structured as a ROS 2 package and should be built from source in a **colcon** workspace. The following steps will guide you through the setup process.

### Create a Colcon Workspace

If you don't have one already, create a new tutorial workspace and navigate to it:

```sh
mkdir -p ~/tutorial_ws/src
cd ~/tutorial_ws/src
```

### Clone the Repository

Download package code into it:

```sh
git clone https://github.com/lexmaister/gazebo_tutorial.git
```

### Install Dependencies with `rosdep`

`rosdep` is a command-line tool that will automatically find and install all the system dependencies your project needs, including MoveIt, Gazebo, and other required ROS 2 packages. This is the recommended way to prepare your environment.

```sh
# Navigate to the workspace root
cd ~/tutorial_ws

# Initialize rosdep (only needs to be done once per new ROS installation)
sudo rosdep init
rosdep update

# Run rosdep to install all declared dependencies from the packages in 'src'
rosdep install --from-paths src -y --ignore-src
```

This command reads the `package.xml` files in your source directory, identifies all dependencies, and installs them using your system's package manager (e.g., `apt`).

### Build and Source the Workspace

Once all dependencies are installed, build the packages using `colcon`:

```sh
# From the workspace root (e.g., ~/tutorial_ws)
colcon build --packages-select panda_gz_moveit panda_mtc_py --symlink-install 

# After the build is complete, source the new setup file
source install/setup.bash
```

Sourcing the `setup.bash` file makes your terminal aware of the packages you just built, so you can launch them.

With these steps completed, you are now ready to launch the simulation.

## Launch Architecture

### Catresian movement using `cartesian.launch.py`

This launch file performs several steps:

1. **Configure MoveIt for Panda**
   It calls `get_panda_moveit_config(use_sim_time=True).to_dict()` from
   `panda_gz_moveit.moveit_config`, producing a dictionary of parameters
   passed to the MTC node.
2. **Launch Gazebo + MoveIt Environment**
   It includes `main.launch.py` from the `panda_gz_moveit` package:
   - Starts Gazebo with the Panda robot model.
   - Loads controllers and `move_group`.
   - Starts RViz configured with `panda_mtc_py/config/rviz_mtc.rviz`:
  
   ```python
   rviz_config_path = PathJoinSubstitution([
       FindPackageShare("panda_mtc_py"),
       "config",
       "rviz_mtc.rviz",
   ])
   ```

3. **Wait for Environment Readiness**
   It starts the `wait_env_ready` node, which blocks until the Panda hand
   controller action server (`/panda_hand_controller/follow_joint_trajectory`)
   is available. Once ready, the node shuts down.
4. **Load Scene and Target from YAML**
   After the helper node exits, the launch file:
   - Starts `add_scene_from_yaml` with `scene_path` set to `config/scene.yaml`.
   - Once the obstacle is confirmed added, starts another `add_scene_from_yaml`
     instance with `scene_path` set to `config/target.yaml`.
   Both instances monitor the `/monitored_planning_scene` topic and shut down
   when the corresponding collision object is present.
5. **Start the MTC Node**
   When the target object has been added successfully, the launch file starts
   the main `mtc_node` with:
   - The `moveit_config` parameters.
   - `use_sim_time=True`.
The logic for starting each step is implemented via `OnProcessExit` handlers:
each node’s completion triggers the next stage in the sequence.

### Running the Simulation and Task

After building and sourcing the workspace, you can run the full pipeline.

1. **Source the Workspace**
   In a new terminal:

   ```sh
   cd ~/tutorial_ws
   source install/setup.bash
   ```

2. **Launch the Cartesian Task Tutorial**
   Launch the main tutorial:

   ```sh
   ros2 launch panda_mtc_py cartesian.launch.py
   ```

3. **What Happens Internally**
   - Gazebo starts with the Panda robot and MoveIt integration
     (from `panda_gz_moveit`).
   - RViz opens with a configuration that displays:
     - The robot model.
     - The planning scene.
   - `wait_env_ready` waits for the Panda hand action server.
   - `add_scene_from_yaml` loads the composite obstacle from `scene.yaml`.
   - Another `add_scene_from_yaml` loads the cylindrical target from `target.yaml`.
   - The `mtc_node` defines and solves the Cartesian task, publishing the
     first solution if planning succeeds.

4. **Observing in RViz**
   In RViz, you should be able to:
   - See the Panda robot, the composite obstacle (`composite_obstacle`) and
     the target (`target`).
   - Visualize the planned Cartesian path.
   - Use standard MoveIt/RViz tools (depending on your configuration) to
     inspect the trajectory and execute it in the simulation.

### Interacting with RViz

The RViz configuration (`rviz_mtc.rviz`) is tailored to this example and
typically includes:

- Robot model and planning scene visualization.
- Visualization of trajectory markers and collision objects.
- Panels for interacting with the PlanningScene and MotionPlanning plugins.

### Video #1: catresian path

[![video_tutorial_YT](https://img.youtube.com/vi/yleyBoTt2XI/0.jpg)](https://youtu.be/yleyBoTt2XI)

## Pick and place task using `pickplace.launch.py`

### Update MTC: use a custom JointInterpolationPlanner

In `mtc_pickplace_node.py`, the `SimpleGrasp` and `SimpleUnGrasp` stages are configured to use an externally provided `JointInterpolationPlanner`. This allows us to run the gripper motion with a reduced speed while keeping the rest of the planning pipeline unchanged. To enable this behavior, you must patch the MoveIt Task Constructor core sources to accept an external planner for these stages and then rebuild MTC from source:

- changes inside `include/moveit/task_constructor/stages/simple_grasp.h`:

```c++
class SimpleGraspBase : public SerialContainer
{
protected:
  // NEW: setup that uses an externally provided planner
  void setup(std::unique_ptr<Stage>&& generator,
             bool forward,
             const std::shared_ptr<solvers::PlannerInterface>& planner);
}

...

class SimpleGrasp : public SimpleGraspBase
{
public:
  // NEW: constructor that accepts an external planner
  SimpleGrasp(std::unique_ptr<Stage>&& generator, const std::shared_ptr<solvers::PlannerInterface>& planner,
              const std::string& name = std::string("grasp"));
};

class SimpleUnGrasp : public SimpleGraspBase
{
public:
  // NEW: constructor that accepts an external planner
  SimpleUnGrasp(std::unique_ptr<Stage>&& generator, const std::shared_ptr<solvers::PlannerInterface>& planner,
                const std::string& name = std::string("ungrasp"));
};
```

- changes inside `src/stages/simple_grasp.cpp`:

```c++
// Use outer planner
void SimpleGraspBase::setup(std::unique_ptr<Stage>&& generator,
                            bool forward,
                            const std::shared_ptr<solvers::PlannerInterface>& planner)

// no need to create an ineer planner
// auto planner = std::make_shared<solvers::JointInterpolationPlanner>();
auto move = new MoveTo(forward ? "close gripper" : "open gripper", planner);

...

SimpleGrasp::SimpleGrasp(std::unique_ptr<Stage>&& generator, const std::shared_ptr<solvers::PlannerInterface>& planner,
                         const std::string& name)
  : SimpleGraspBase(name) {
  // new behavior – uses outer planner
  setup(std::move(generator), true, planner);
}

SimpleUnGrasp::SimpleUnGrasp(std::unique_ptr<Stage>&& generator,
                             const std::shared_ptr<solvers::PlannerInterface>& planner, const std::string& name)
  : SimpleGraspBase(name) {
  // new behavior – uses outer planner
  setup(std::move(generator), false, planner);
}
```

- changes in python binding `core/python/bindings/src/stages.cppstages.cpp`:

```c++

using PlannerPtr = std::shared_ptr<solvers::PlannerInterface>;

properties::class_<SimpleGrasp, SimpleGraspBase>(m, "SimpleGrasp", R"(
    Specialization of SimpleGraspBase to realize grasping.
    ...
  )")
  .def(py::init<Stage::pointer&&, const PlannerPtr&, const std::string&>(),
       "pose_generator"_a,
       "planner"_a,
       "name"_a = std::string("grasp"));

properties::class_<SimpleUnGrasp, SimpleGraspBase>(m, "SimpleUnGrasp", R"(
    Specialization of SimpleGraspBase to realize ungrasping
    ...
  )")
  .property<std::string>("pregrasp", "str: Name of the pre-grasp pose")
  .property<std::string>("grasp", "str: Name of the grasp pose")
  .def(py::init<Stage::pointer&&, const PlannerPtr&, const std::string&>(),
       "pose_generator"_a,
       "planner"_a,
       "name"_a = std::string("ungrasp"));

```

- changes in `core/test/pick_ur5.cpp` and `core/test/pick_pr2.cpp`:

```c++
// add
#include <moveit/task_constructor/solvers/joint_interpolation.h>

namespace mtc = moveit::task_constructor;

// change
auto grasp_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
auto grasp = std::make_unique<mtc::stages::SimpleGrasp>(
    std::move(grasp_generator), grasp_planner, "grasp");
```

Build modified MTC core package (`jazzy` example):

```sh
# inside moveit workspace
cd moveit_ws/
. /opt/ros/jazzy/setup.bash
colcon build --packages-select moveit_task_constructor_core
. install/setup.bash
```

After building and sourcing the MoveIt workspace, an external planner might be used for the grasp stages:

```python
simpleGrasp = stages.SimpleGrasp(grasp_generator, slow_gripper, "Grasp")
...
simpleUnGrasp = stages.SimpleUnGrasp(place_generator, slow_gripper, "UnGrasp")
```

Then you can run the full pipeline:

1. **Source the Gazebo Tutorial Workspace**
   In a new terminal:

   ```sh
   cd ~/tutorial_ws
   source install/setup.bash
   ```

2. **Launch the Pick and Place Task Tutorial**
   Launch the main tutorial:

   ```sh
   ros2 launch panda_mtc_py pickplace.launch.py
   ```

   It is also possible to launch with debug logger level:

   ```sh
   ros2 launch panda_mtc_py pickplace.launch.py logger_level:=debug
   ```

### Video #2: pick and place task

[![video_tutorial_YT](https://img.youtube.com/vi/wZAqsriDVeI/0.jpg)](https://youtu.be/wZAqsriDVeI)

## References

- [MoveIt Task Constructor concepts](https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor/moveit_task_constructor.html)
- [MoveIt Task Constructor documentation (python)](https://moveit.github.io/moveit_task_constructor/index.html)
- [MoveIt Task Constructor GitHub (jazzy)](https://github.com/moveit/moveit_task_constructor/tree/jazzy)
- [Quaternion fundamentals (jazzy)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html)
- [General Gazebo Tutorials Collection](https://github.com/lexmaister/gazebo_tutorial)

## Similar projects

- [Getting Started with the MoveIt 2 Task Constructor](https://automaticaddison.com/getting-started-with-the-moveit-2-task-constructor/)
