from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PKG = "panda_gz_moveit"


def generate_launch_description():
    wall = PathJoinSubstitution([FindPackageShare(PKG), "sdf", "wall.sdf"])

    gz_spawn_wall = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file",
            wall,
            "-name",
            "wall",
            "-x",
            "0.0",
            "-y",
            "-0.5",
            "-z",
            "1.0",
        ],
    )

    return LaunchDescription([gz_spawn_wall])
