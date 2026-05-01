from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from pathlib import Path
import os


def _get_camera_binary(pkg_share: str) -> str:
    """Resolve camera binary path.

    Prefer the installed prefix for `autonomous_kart_cpp` if available;
    fall back to deriving the sibling install layout relative to `pkg_share`.
    """
    try:
        prefix = get_package_prefix("autonomous_kart_cpp")
        candidate = Path(prefix) / "lib" / "autonomous_kart_cpp" / "camera_node"
        if candidate.exists():
            return str(candidate)
    except Exception:
        pass

    # fallback derived path (works in typical overlay layout)
    return str(
        Path(pkg_share).resolve().parents[2]
        / "autonomous_kart_cpp"
        / "lib"
        / "autonomous_kart_cpp"
        / "camera_node"
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("autonomous_kart")
    camera_binary = _get_camera_binary(pkg_share)

    motor_yaml = os.path.join(pkg_share, "params", "motor.yaml")
    steering_yaml = os.path.join(pkg_share, "params", "steering.yaml")
    camera_yaml = os.path.join(pkg_share, "params", "camera.yaml")
    gps_yaml = os.path.join(pkg_share, "params", "gps.yaml")
    safety_yaml = os.path.join(pkg_share, "params", "safety.yaml")
    system_yaml = os.path.join(pkg_share, "params", "system.yaml")
    pathfinder_yaml = os.path.join(pkg_share, "params", "pathfinder.yaml")
    localization_yaml = os.path.join(pkg_share, "params", "localization.yaml")

    sim_mode = LaunchConfiguration("simulation_mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument("simulation_mode", default_value="true"),
            GroupAction(
                [
                    SetParametersFromFile(system_yaml),
                    SetParameter(name="simulation_mode", value=sim_mode),
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
                            "--params-file",
                            system_yaml,
                        ],
                        output="screen",
                    ),
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
                        parameters=[gps_yaml],
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
                        parameters=[pathfinder_yaml, localization_yaml],
                    ),
                ]
            ),
        ]
    )
