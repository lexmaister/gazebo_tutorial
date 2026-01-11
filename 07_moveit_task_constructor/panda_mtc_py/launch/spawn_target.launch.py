from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PKG = "panda_mtc_py"


def generate_launch_description():
    target = PathJoinSubstitution([FindPackageShare(PKG), "sdf", "target.sdf"])

    gz_spawn_target = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file",
            target,
            "-name",
            "target",
            "-x",
            "0.45",
            "-y",
            "0.3",
            "-z",
            "0.36",
        ],
    )

    return LaunchDescription([gz_spawn_target])
