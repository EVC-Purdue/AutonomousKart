from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("autonomous_kart")

    motor_yaml = os.path.join(pkg_share, "params", "motor.yaml")
    steering_yaml = os.path.join(pkg_share, "params", "steering.yaml")
    gps_yaml = os.path.join(pkg_share, "params", "gps.yaml")
    safety_yaml = os.path.join(pkg_share, "params", "safety.yaml")
    system_yaml = os.path.join(pkg_share, "params", "system.yaml")
    planner_yaml = os.path.join(pkg_share, "params", "pathfinder.yaml")

    manual = LaunchConfiguration("manual")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "manual",
                default_value="true",
            ),
            GroupAction(
                [
                    SetParametersFromFile(system_yaml),

                    # Manual mode flag
                    SetParameter(
                        name="manual",
                        value=ParameterValue(manual, value_type=bool),
                    ),

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
                        executable="gps_node",
                        name="gps_node",
                        parameters=[gps_yaml],
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
                        package='autonomous_kart',
                        executable='master_api',
                        name='master_api',
                    ),
                ]
            ),
        ]
    )
