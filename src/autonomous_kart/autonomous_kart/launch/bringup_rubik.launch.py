from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("autonomous_kart")

    camera_yaml = os.path.join(pkg_share, "params", "camera.yaml")
    system_yaml = os.path.join(pkg_share, "params", "system.yaml")

    sim_mode = LaunchConfiguration("simulation_mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument("simulation_mode", default_value="false"),
            GroupAction(
                [
                    SetParametersFromFile(system_yaml),
                    SetParameter(name="simulation_mode", value=sim_mode),

                    Node(
                        package="autonomous_kart",
                        executable="camera_node",
                        name="camera_node",
                        parameters=[camera_yaml],
                    ),
                ]
            ),
        ]
    )
