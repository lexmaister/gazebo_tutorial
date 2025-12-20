from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from panda_gz_moveit.moveit_config import get_panda_moveit_config


PKG = "panda_mtc_py"
ENV_PKG = "panda_gz_moveit"


def generate_launch_description():
    moveit_config = get_panda_moveit_config(use_sim_time=True).to_dict()

    # --- Launch env (Gazebo + controllers + move_group) ---
    env_launch_path = PathJoinSubstitution(
        [
            FindPackageShare(ENV_PKG),
            "launch",
            "main.launch.py",
        ]
    )

    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare(PKG),
            "config",
            "rviz_mtc.rviz",
        ]
    )

    env_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(env_launch_path),
        launch_arguments={
            "rviz_config": rviz_config_path,
        }.items(),
    )

    # --- Helper node: waits until panda_hand_controller is ready, then exits ---
    env_waiter = Node(
        package=PKG,
        executable="wait_env_ready",
        name="wait_env_ready",
        output="screen",
    )

    # --- Loading scene ---

    scene_yaml_path = PathJoinSubstitution(
        [
            FindPackageShare(PKG),
            "config",
            "scene.yaml",
        ]
    )

    target_yaml_path = PathJoinSubstitution(
        [
            FindPackageShare(PKG),
            "config",
            "target.yaml",
        ]
    )

    add_scene = Node(
        package=PKG,
        executable="add_scene_from_yaml",
        name="add_scene",
        output="screen",
        parameters=[{"scene_path": scene_yaml_path}],
    )

    add_target = Node(
        package=PKG,
        executable="add_scene_from_yaml",
        name="add_target",
        output="screen",
        parameters=[{"scene_path": target_yaml_path}],
    )

    # --- Main MTC node ---
    cartesian_task = Node(
        package=PKG,
        executable="mtc_node",
        output="screen",
        parameters=[moveit_config, {"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            env_launch,
            env_waiter,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=env_waiter,
                    on_exit=[add_scene],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=add_scene,
                    on_exit=[add_target],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=add_target,
                    on_exit=[cartesian_task],
                )
            ),
        ]
    )
