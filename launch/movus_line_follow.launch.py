import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_movus_ignition = get_package_share_directory("movus_ignition_sim")
    pkg_ros_ign_gazebo = get_package_share_directory("ros_ign_gazebo")

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use sim time",
    )

    movus_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_movus_ignition, "launch", "movus.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_track = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=[
            "-name",
            "line_track",
            "-x",
            "5.0",
            "-y",
            "10.0",
            "-z",
            "0.18",
            "-R",
            "0.0",
            "-Y",
            "0.0",
            "-file",
            os.path.join(pkg_movus_ignition, "models", "line_follower_track", "model.sdf"),
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(spawn_track)

    ld.add_action(movus_launch)

    return ld
