from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from panda_gz_moveit.moveit_config import get_panda_moveit_config

PKG = "panda_gz_moveit"


def generate_launch_description():
    #  ----- PARAMETERS -----

    logger_level = LaunchConfiguration("logger_level")

    # use config builder inside separate module to share with other packages
    moveit_config = get_panda_moveit_config(use_sim_time=True)

    ros2_controllers = PathJoinSubstitution(
        [FindPackageShare(PKG), "config", "ros2_controllers.yaml"]
    )

    world = PathJoinSubstitution([FindPackageShare(PKG), "sdf", "sim_world.sdf"])

    rviz_config = PathJoinSubstitution([FindPackageShare(PKG), "config", "moveit.rviz"])

    # ----- LAUNCH ARGUMENTS -----

    logger_level_arg = DeclareLaunchArgument(
        "logger_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, critical)",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_config,
        description="Path to RViz2 config file",
    )

    # ----- NODES -----

    # Robots description publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        output="screen",
        arguments=["--ros-args", "--log-level", logger_level],
    )

    # move_group node (THE CORE OF MOVEIT)
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # load ExecuteTaskSolutionCapability so we can execute found MTC solutions in simulation
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
            # tell the MoveIt motion planning node (inside ROS 2) to use simulation time
            {"use_sim_time": True},
        ],
        arguments=["--ros-args", "--log-level", logger_level],
    )

    # RViz (with MoveIt plugins)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="panda_rviz",
        output="screen",
        arguments=[
            "-d",
            LaunchConfiguration("rviz_config"),
            "--ros-args",
            "--log-level",
            logger_level,
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    # Gasebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ]
        ),
        # engine for mimic joint - https://github.com/ros-controls/gz_ros2_control/issues/340
        launch_arguments=[
            (
                "gz_args",
                [
                    " -r -v 2 ",
                    world,
                    " --physics-engine gz-physics-bullet-featherstone-plugin",
                ],
            )
        ],
    )

    gz_spawn_panda = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "panda",
            "-allow_renaming",
            "true",
            "--ros-args",
            "--log-level",
            logger_level,
        ],
    )

    # ----- CONTROLLERS -----

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--param-file",
            ros2_controllers,
            "--ros-args",
            "--log-level",
            logger_level,
        ],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_arm_controller",
            "--param-file",
            ros2_controllers,
            "--ros-args",
            "--log-level",
            logger_level,
        ],
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_hand_controller",
            "--param-file",
            ros2_controllers,
            "--ros-args",
            "--log-level",
            logger_level,
        ],
    )

    spawn_chain = (
        gz_spawn_panda,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        panda_hand_controller_spawner,
    )

    spawners = []
    for i in range(len(spawn_chain) - 1):
        spawners.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_chain[i],
                    on_exit=spawn_chain[i + 1],
                )
            )
        )

    # Clock bridge: Gazebo <-> ROS 2
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "--ros-args",
            "--log-level",
            logger_level,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            logger_level_arg,
            rviz_config_arg,
            run_move_group_node,
            gz_sim,
            bridge,
            node_robot_state_publisher,
            gz_spawn_panda,
            rviz_node,
        ]
        + spawners
    )
