from launch import LaunchDescription
from launch.actions import GroupAction, ExecuteProcess
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from pathlib import Path
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("autonomous_kart")
    camera_binary = str(Path(pkg_share).resolve().parents[2] / "autonomous_kart_cpp" / "lib" / "autonomous_kart_cpp" / "camera_node")

    motor_yaml = os.path.join(pkg_share, "params", "motor.yaml")
    steering_yaml = os.path.join(pkg_share, "params", "steering.yaml")
    camera_yaml = os.path.join(pkg_share, "params", "camera.yaml")
    gps_yaml = os.path.join(pkg_share, "params", "gps.yaml")
    safety_yaml = os.path.join(pkg_share, "params", "safety.yaml")
    system_yaml = os.path.join(pkg_share, "params", "system.yaml")
    planner_yaml = os.path.join(pkg_share, "params", "pathfinder.yaml")
    localization_yaml = os.path.join(pkg_share, "params", "localization.yaml")

    return LaunchDescription(
        [
            GroupAction(
                [
                    SetParametersFromFile(system_yaml),
                    # Force simulator mode OFF for the real kart.
                    SetParameter(name="simulation_mode", value=False),
                    Node(
                        package="autonomous_kart",
                        executable="motor_node",
                        name="motor_node",
                        parameters=[motor_yaml],
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="steering_node",
                        name="steering_node",
                        parameters=[steering_yaml],
                    ),
                    ExecuteProcess(
                        cmd=[
                            str(camera_binary),
                            "--ros-args",
                            "-r",
                            "__node:=camera_node",
                            "--params-file",
                            camera_yaml,
                        ],
                        output="screen",
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="gps_node",
                        name="gps_node",
                        parameters=[gps_yaml],
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="pathfinder_node",
                        name="pathfinder_node",
                        parameters=[planner_yaml, safety_yaml],
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="opencv_pathfinder_node",
                        name="opencv_pathfinder_node",
                        parameters=[],
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="e_comms_node",
                        name="e_comms_node",
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="metrics_node",
                        name="metrics_node",
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="master_api",
                        name="master_api",
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="localization_node",
                        name="localization_node",
                        parameters=[planner_yaml, localization_yaml],
                    ),
                ]
            ),
        ]
    )
