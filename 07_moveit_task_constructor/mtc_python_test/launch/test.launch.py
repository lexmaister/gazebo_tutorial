import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_gz_moveit")
        .robot_description(
            file_path="urdf/panda.urdf.xacro",
            # IMPORTANT: Use sim time to sync ros2 <-> gazebo
            # tell the robot hardware simulation plugin (inside Gazebo) to use simulation time
            mappings={"use_sim_time": "true"},
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
        .to_dict()
    )

    test_task = Node(
        package="mtc_python_test",
        executable="mtc_node",
        output="screen",
        parameters=[moveit_config, {"use_sim_time": True}],
    )

    return LaunchDescription([test_task])
