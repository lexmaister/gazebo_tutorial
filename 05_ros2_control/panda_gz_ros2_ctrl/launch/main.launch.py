from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


# "Could not find shared library" error
# https://github.com/ros-controls/gz_ros2_control/issues/390


def generate_launch_description():

    # ----- PARAMETERS -----

    # Launch Arguments
    logger_level_arg = DeclareLaunchArgument(
        'logger_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )
    logger_level = LaunchConfiguration('logger_level')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot description from URDF
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('panda_gz_ros2_ctrl'),
            'panda_description',
            'urdf',
            'panda.urdf.xacro'
        ])
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # Controller config path
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('panda_gz_ros2_ctrl'),
        'config',
        'controller.yaml'
    ])

    # World SDF path
    world = PathJoinSubstitution([
        FindPackageShare('panda_gz_ros2_ctrl'),
        'sdf',
        'main.sdf'
    ])

    # RViz config path
    rviz_config = PathJoinSubstitution([
        FindPackageShare('panda_gz_ros2_ctrl'),
        'config',
        'view_robot.rviz'
    ])

    # ----- NODES -----

    # Robots description publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen',
        arguments=['--ros-args', '--log-level', logger_level]
    )

    # Gasebo Sim
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'panda', 
            '-allow_renaming', 'true',
            '--ros-args', '--log-level', logger_level],
    )

    # Controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster', 
            '--param-file', robot_controllers,
            '--ros-args', '--log-level', logger_level
            ],
    )

    panda_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'panda_arm_controller', 
            '--param-file', robot_controllers,
            '--ros-args', '--log-level', logger_level
            ],
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_hand_controller", 
            '--param-file', robot_controllers,
            '--ros-args', '--log-level', logger_level],
    )

    # Clock bridge: Gazebo <-> ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', 
                   '--ros-args', '--log-level', logger_level],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', logger_level],
        # RViz needs to use simulation time
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        logger_level_arg,
        # Launch Gazebo with specified world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
            ]),
            # engine for mimic joint - https://github.com/ros-controls/gz_ros2_control/issues/340
            launch_arguments=[
                ('gz_args', [
                    ' -r -v 2 ', 
                    world, 
                    ' --physics-engine gz-physics-bullet-featherstone-plugin',
                    ]
                )]
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[panda_arm_controller_spawner, panda_hand_controller_spawner]
            )
        ),
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        rviz_node,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use simulated clock'
        ),
    ])