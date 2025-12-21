from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_kart')

    motor_yaml = os.path.join(pkg_share, 'params', 'motor.yaml')
    steering_yaml = os.path.join(pkg_share, 'params', 'steering.yaml')
    camera_yaml = os.path.join(pkg_share, 'params', 'camera.yaml')
    gps_yaml = os.path.join(pkg_share, 'params', 'gps.yaml')
    safety_yaml = os.path.join(pkg_share, 'params', 'safety.yaml')
    system_yaml = os.path.join(pkg_share, 'params', 'system.yaml')
    pathfinder_yaml = os.path.join(pkg_share, 'params', 'pathfinder.yaml')

    sim_mode = LaunchConfiguration('simulation_mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='true'
        ),

        GroupAction([
            SetParametersFromFile(system_yaml),
            SetParameter(name='simulation_mode', value=sim_mode),

            Node(
                package='autonomous_kart',
                executable='motor_node',
                name='motor_node',
                parameters=[motor_yaml],
            ),
            Node(
                package='autonomous_kart',
                executable='steering_node',
                name='steering_node',
                parameters=[steering_yaml],
            ),
            Node(
                package='autonomous_kart',
                executable='camera_node',
                name='camera_node',
                parameters=[camera_yaml],
            ),
            Node(
                package='autonomous_kart',
                executable='pathfinder_node',
                name='pathfinder_node',
                parameters=[pathfinder_yaml, safety_yaml],
            ),
            Node(
                package='autonomous_kart',
                executable='opencv_pathfinder_node',
                name='opencv_pathfinder_node',
                parameters=[gps_yaml],
            ),
            Node(
                package='autonomous_kart',
                executable='metrics_api',
                name='metrics_api',
            ),
            Node(
                package='autonomous_kart',
                executable='master_api',
                name='master_api',
            ),
        ]),
    ])
