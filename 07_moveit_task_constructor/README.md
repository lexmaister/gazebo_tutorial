# Python

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
