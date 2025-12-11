# C++

From [official tutorial](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)

## Create package

```sh
ros2 pkg create mtc_pick_place --node-name mtc_node --build-type ament_cmake --dependencies rclcpp moveit_ros_planning moveit_task_constructor_core moveit_ros_planning_interface tf2_geometry_msgs tf2_eigen --license MIT --maintainer-email lexmaister@gmail.com --maintainer-name lexmaister --description "Tutorial package with basic MoveIt Task Constructor usage in pick and place operation"
```

### Verify Build Configuration

After creating your package, it's crucial to verify that the `package.xml` and `CMakeLists.txt` files are correctly configured with all necessary dependencies. An incorrect or incomplete configuration is the most common source of build errors like "file not found."

- check `package.xml`

Ensure this file contains `<depend>` tags for all required packages. These tags inform the ROS 2 build system what other packages yours relies on.

```xml
<package>
  <!-- ... other tags like license, author, etc. ... -->

  <build_depend>ament_cmake</build_depend>

  <depend>rclcpp</depend>
  <depend>moveit_ros_planning</depend>
  <depend>moveit_task_constructor_core</depend>
  <depend>moveit_ros_planning_interface</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>tf2_eigen</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- Check `CMakeLists.txt`

This file tells CMake how to build your ```. It needs to both find the dependency packages and link them to your executable.

First, verify that the `find_package` commands are present for all dependencies. It is best practice to add the `REQUIRED` keyword.

```sh
cmake_minimum_required(VERSION 3.8)
project(mtc_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
```

Next, ensure that your executable is linked against these packages using `ament_target_dependencies`. This is the step that makes the headers and libraries available to your ```.

```sh
# ... (add_executable and other commands) ...

# Link the executable to the required libraries
ament_target_dependencies(mtc_node
  "rclcpp"
  "moveit_ros_planning"
  "moveit_task_constructor_core"
  "moveit_ros_planning_interface"
  "tf2_geometry_msgs"
  "tf2_eigen"
)

# ... (install rules and other commands) ...
```

By ensuring these configurations are correct, you can avoid most common build-related issues with ROS 2 and MoveIt.

## PYTHON - failed

### outline - conda

<!-- https://github.com/ros-planning/py_binding_tools
https://github.com/moveit/moveit_task_constructor/issues/520#issuecomment-1979433917 -->

```sh
sudo apt update
sudo apt install ros-$ROS_DISTRO-py-binding-tools
```

- create conda env and workspace dir

```sh
conda conda create -n moveit_task_constructor python=3.12
conda activate moveit_task_constructor
pip install transforms3d
mkdir -p tutorial_ws/src
cd tutorial_ws/src
```

- create pkg
  
```sh
ros2 pkg create panda_mtc_py --build-type ament_python --dependencies rclpy rclcpp moveit_configs_utils moveit_task_constructor_core --license MIT --maintainer-email lexmaister@gmail.com --maintainer-name lexmaister --description "Tutorial python package with an example of basic MoveIt Task Constructor's API usage for pick and place operation"
```

- update `setup.py`

```python
...
data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files from the 'launch' and 'config' directories
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
],

...
entry_points={
        "console_scripts": [
            f"mtc_node = {package_name}.mtc_node:main",
        ],
},
...
```

- add `launch/pick_place.launch.py`
