from setuptools import setup

package_name = 'state'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='State manager, lifecycle base, safety monitor, teleop',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_manager_node = state.state_manager_node:main',
            'safety_monitor_node = state.safety_monitor_node:main',
            'teleop_node = state.teleop_node:main',
        ],
    },
)
