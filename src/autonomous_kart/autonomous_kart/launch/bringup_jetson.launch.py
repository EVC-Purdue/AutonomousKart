"""Distributed role: Jetson. Compute + sensors physically on this board
(GPS/IMU/CAN) plus the CV compute (opencv_pathfinder_node subscribes to
camera/image_raw over the network from the Rubik Pi running camera_node --
see bringup_rubik.launch.py). Same node set as bringup_pi.launch.py, split
across boards instead of one process; params here are kept in sync with
bringup_pi.launch.py's per-node lists, not the older (now-stale)
millan/simple_distribute branch's versions.

Run via `TARGET_DEVICE=jetson` (compose/docker-compose.yml) or directly:
`scripts/kart jetson`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("autonomous_kart")

    gps_yaml = os.path.join(pkg_share, "params", "gps.yaml")
    safety_yaml = os.path.join(pkg_share, "params", "safety.yaml")
    system_yaml = os.path.join(pkg_share, "params", "system.yaml")
    pathfinder_yaml = os.path.join(pkg_share, "params", "pathfinder.yaml")
    opencv_yaml = os.path.join(pkg_share, "params", "opencv_pathfinder.yaml")
    imu_yaml = os.path.join(pkg_share, "params", "imu.yaml")
    e_comms_yaml = os.path.join(pkg_share, "params", "e_comms.yaml")
    actuators_yaml = os.path.join(pkg_share, "params", "actuators.yaml")
    localization_yaml = os.path.join(pkg_share, "params", "localization.yaml")

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
                        executable="pathfinder_node",
                        name="pathfinder_node",
                        parameters=[pathfinder_yaml, safety_yaml],
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="opencv_pathfinder_node",
                        name="opencv_pathfinder_node",
                        parameters=[gps_yaml, system_yaml, opencv_yaml],
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="localization_node",
                        name="localization_node",
                        parameters=[localization_yaml],
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="e_comms_node",
                        name="e_comms_node",
                        parameters=[e_comms_yaml, actuators_yaml, pathfinder_yaml],
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="gps_node",
                        name="gps_node",
                        parameters=[gps_yaml],
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="imu_node",
                        name="imu_node",
                        parameters=[imu_yaml],
                    ),
                ]
            ),
        ]
    )
