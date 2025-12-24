from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("autonomous_kart")

    motor_yaml = os.path.join(pkg_share, "params", "motor.yaml")
    steering_yaml = os.path.join(pkg_share, "params", "steering.yaml")
    camera_yaml = os.path.join(pkg_share, "params", "camera.yaml")
    gps_yaml = os.path.join(pkg_share, "params", "gps.yaml")
    safety_yaml = os.path.join(pkg_share, "params", "safety.yaml")
    system_yaml = os.path.join(pkg_share, "params", "system.yaml")
    planner_yaml = os.path.join(pkg_share, "params", "pathfinder.yaml")

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
                    Node(
                        package="autonomous_kart",
                        executable="camera_node",
                        name="camera_node",
                        parameters=[camera_yaml],
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
                        parameters=[planner_yaml, safety_yaml, gps_yaml],
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="opencv_pathfinder_node",
                        name="opencv_pathfinder_node",
                        parameters=[planner_yaml, safety_yaml, gps_yaml],
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="e_comms_node",
                        name="e_comms_node",
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="metrics_api",
                        name="metrics_api",
                    ),
                    Node(
                        package="autonomous_kart",
                        executable="master_api",
                        name="master_api",
                    ),
                ]
            ),
        ]
    )
