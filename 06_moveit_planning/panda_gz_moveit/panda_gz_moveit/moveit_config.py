from moveit_configs_utils import MoveItConfigsBuilder, MoveItConfigs


def get_panda_moveit_config(use_sim_time: bool = True) -> MoveItConfigs:
    """
    Build and return a reusable MoveIt 2 configuration for the Panda manipulator
    in the Gazebo–ROS 2 tutorial environment.

    This helper centralizes all MoveIt configuration for the Panda robot
    (URDF, SRDF, kinematics, joint limits, controllers, and planning pipelines)
    so it can be consistently reused across multiple launch files
    (e.g. environment / simulation launch, MTC demos, custom RViz setups).

    The function wraps a `MoveItConfigsBuilder("panda", package_name=PKG)` call
    with a fixed set of file paths that are expected to live in the
    MoveIt configuration package `PKG`, such as:

    - `urdf/panda.urdf.xacro` for the robot description
    - `srdf/panda.srdf` for the semantic description
    - `config/kinematics.yaml` for kinematics solver parameters
    - `config/joint_limits.yaml` for joint limits
    - `config/moveit_controllers.yaml` for trajectory execution and controllers

    The resulting object can be used directly as:

    - `moveit_config.robot_description` (URDF parameters)
    - `moveit_config.robot_description_semantic`
    - `moveit_config.robot_description_kinematics`
    - `moveit_config.joint_limits`
    - `moveit_config.trajectory_execution`
    - `moveit_config.planning_pipelines`
    - or as a single dictionary via `moveit_config.to_dict()` to pass
      into MoveIt nodes (e.g. `move_group`, planning scene monitor,
      RViz2, custom MTC nodes).

    Parameters
    ----------
    use_sim_time : bool, optional
        If `True`, the generated `robot_description` mapping will set
        `"use_sim_time"` to `"true"`, which makes the Panda model and
        MoveIt stack use simulation time. This is required when the robot
        is driven from Gazebo (or another simulated clock) so that
        controllers, planning scene updates, and trajectories stay
        synchronized with the simulator.
        If `False`, `"use_sim_time"` is set to `"false"` and the MoveIt
        configuration will expect wall-clock ROS time instead.

    Returns
    -------
    moveit_configs_utils.MoveItConfigs
        A fully constructed MoveIt 2 configuration object for the Panda
        manipulator, ready to be used in ROS 2 launch files. The object
        exposes individual config dictionaries (e.g.
        `robot_description`, `robot_description_semantic`, etc.) and a
        convenience `to_dict()` method that merges all of them for use
        as node parameters.

    Notes
    -----
    - This function is intended to be imported and used by multiple
      launch files (e.g. `main.launch.py` for the Gazebo environment
      and `mtc_demo.launch.py` for MoveIt Task Constructor examples),
      ensuring that all of them share *exactly the same* MoveIt setup.
    - Keeping the builder logic in one place avoids configuration drift
      between different demos and makes it easier to update the Panda
      MoveIt configuration in the future.
    """
    return (
        MoveItConfigsBuilder("panda", package_name="panda_gz_moveit")
        .robot_description(
            file_path="urdf/panda.urdf.xacro",
            # IMPORTANT: Use sim time to sync ros2 <-> gazebo
            # tell the robot hardware simulation plugin (inside Gazebo) to use simulation time
            mappings={"use_sim_time": "true" if use_sim_time else "false"},
        )
        .robot_description_semantic(file_path="srdf/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=False,  # due to custom semantic was specified
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )
