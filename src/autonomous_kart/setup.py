from glob import glob
from setuptools import setup, find_packages

package_name = 'autonomous_kart'

setup(
    name=package_name,
    version="0.0.1",
    packages=[
        'autonomous_kart',
        'autonomous_kart.nodes',
        'autonomous_kart.nodes.motor',
        'autonomous_kart.nodes.steering',
        'autonomous_kart.nodes.camera',
        'autonomous_kart.nodes.gps',
        'autonomous_kart.nodes.pathfinder',
        'autonomous_kart.nodes.opencv_pathfinder',
        'autonomous_kart.nodes.3dgs_localization',
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("autonomous_kart/launch/*.py")),  # Fixed path
        ("share/" + package_name + "/params", glob("autonomous_kart/params/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer='Purdue EVC',
    maintainer_email='shay.manor@gmail.com',
    license='Apache-2.0',
    description="Package containing all nodes for driving in different states.",
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'motor_node = autonomous_kart.nodes.motor.motor_node:main',
            'pathfinder_node = autonomous_kart.nodes.pathfinder.pathfinder_node:main',
            'gps_node = autonomous_kart.nodes.gps.gps_node:main',
            '3dgs_localization_node = autonomous_kart.nodes.3dgs_localization.3dgs_localization_node:main',
            'opencv_pathfinder_node = autonomous_kart.nodes.opencv_pathfinder.opencv_pathfinder_node:main',
            'steering_node = autonomous_kart.nodes.steering.steering_node:main',
            'camera_node = autonomous_kart.nodes.camera.camera_node:main',
        ],
    },
)