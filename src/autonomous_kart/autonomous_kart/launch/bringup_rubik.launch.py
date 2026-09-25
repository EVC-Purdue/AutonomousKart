"""Distributed role: Rubik Pi. Camera capture + the HTTP/telemetry layer +
metrics -- physically closest to the camera mount; opencv_pathfinder_node
(the CV compute consuming camera/image_raw) runs on the Jetson instead, see
bringup_jetson.launch.py. Same node set as bringup_pi.launch.py, split
across boards; master_api's params are kept in sync with
bringup_pi.launch.py's current list (the older millan/simple_distribute
branch launched master_api with no params at all, which predates
master_node needing pathfinder_yaml/gps_yaml -- don't copy that).

Run via `TARGET_DEVICE=rubik` (compose/docker-compose.yml) or directly:
`scripts/kart rubik`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("autonomous_kart")

    camera_yaml = os.path.join(pkg_share, "params", "camera.yaml")
    gps_yaml = os.path.join(pkg_share, "params", "gps.yaml")
    system_yaml = os.path.join(pkg_share, "params", "system.yaml")
    pathfinder_yaml = os.path.join(pkg_share, "params", "pathfinder.yaml")

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
                    Node(
                        package="autonomous_kart",
                        executable="master_api",
                        name="master_api",
                        parameters=[pathfinder_yaml, gps_yaml],
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="metrics_node",
                        name="metrics_node",
                    ),
                ]
            ),
        ]
    )
