"""Launch diffdrive_bot simulation and run square driving action client and rotate server.

Includes control.launch.py from diffdrive_bot, starts rotate_server and square_driver.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    diffdrive_bot_share = get_package_share_directory("diffdrive_bot")

    side_length = ParameterValue(LaunchConfiguration("side_length"), value_type=float)

    # Include diffdrive_bot's control.launch.py (which includes Gazebo, bridge, RViz and default nodes)
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(diffdrive_bot_share, "launch", "control.launch.py")),
    )

    # Rotate action server
    rotate_server_node = Node(
        package="my_robot_nodes",
        executable="rotate_server",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Square driver action client
    square_driver_node = Node(
        package="my_robot_nodes",
        executable="square_driver",
        parameters=[{"side_length": side_length, "use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "side_length",
            default_value="0.5",
            description="Side length of the driven square in meters (max 1.0 m)",
        ),
        control_launch,
        rotate_server_node,
        square_driver_node,
    ])
