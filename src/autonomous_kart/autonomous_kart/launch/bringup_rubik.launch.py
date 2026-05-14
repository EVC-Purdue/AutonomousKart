from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("autonomous_kart")
    system_yaml = os.path.join(pkg_share, "params", "system.yaml")
    run_name = datetime.now().strftime("run_%Y%m%d_%H%M%S")
    bag_output_path = os.path.join("/ws/logs", run_name)
    sim_mode = LaunchConfiguration("simulation_mode")
    rosbag_recorder = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record', '-a',
                '--storage', 'mcap',
                '--storage-preset-profile', 'zstd_fast',
                '--max-bag-duration', '60',
                '--exclude', '/camera/.*',
                '--output', bag_output_path
            ],
            output='screen',
            respawn=True,
            respawn_delay=1.0
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("simulation_mode", default_value="false"),
            GroupAction(
                [
                    SetParametersFromFile(system_yaml),
                    SetParameter(name="simulation_mode", value=sim_mode),

                    rosbag_recorder,

                    Node(
                        package="autonomous_kart",
                        executable="metrics_node",
                        name="metrics_node",
                    ),
                ]
            ),
        ]
    )
