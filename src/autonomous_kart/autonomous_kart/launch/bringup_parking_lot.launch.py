from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("autonomous_kart")

    return LaunchDescription(
        [
            Node(
                package="autonomous_kart",
                executable="motor_node",
                name="motor_node",
                parameters=[
                    os.path.join(pkg_share, "params", "controller.yaml"),
                    {"simulation_mode": False},
                    {"parking_lot_mode": True},
                ],
            ),
            Node(
                package="autonomous_kart",
                executable="steering_node",
                name="steering_node",
                parameters=[
                    os.path.join(pkg_share, "params", "controller.yaml"),
                    {"parking_lot_mode": True},
                ],
            ),
            Node(
                package="autonomous_kart",
                executable="pathfinder_node",
                name="pathfinder_node",
                parameters=[
                    os.path.join(pkg_share, "params", "planner.yaml"),
                    os.path.join(pkg_share, "params", "safety.yaml"),
                    os.path.join(pkg_share, "params", "gps.yaml"),
                    {"parking_lot_mode": True},
                ],
            ),
        ]
    )
