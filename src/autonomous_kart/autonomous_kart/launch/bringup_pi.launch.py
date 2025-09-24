from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_kart')

    return LaunchDescription([
        # Hardware nodes for real kart
        Node(
            package='autonomous_kart',
            executable='motor_node',
            name='motor_node',
            parameters=[os.path.join(pkg_share, 'params', 'controller.yaml')]
        ),
        Node(
            package='autonomous_kart',
            executable='steering_node',
            name='steering_node',
            parameters=[os.path.join(pkg_share, 'params', 'controller.yaml')]
        ),
        Node(
            package='autonomous_kart',
            executable='camera_node',
            name='camera_node',
            parameters=[os.path.join(pkg_share, 'params', 'camera.yaml')]
        ),
        Node(
            package='autonomous_kart',
            executable='gps_node',
            name='gps_node'
        ),
        Node(
            package='autonomous_kart',
            executable='pathfinder_node',
            name='pathfinder_node',
            parameters=[os.path.join(pkg_share, 'params', 'planner.yaml')]
        ),
    ])