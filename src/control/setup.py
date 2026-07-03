from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        (os.path.join('share', package_name, 'database'), glob('control/database/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alkady',
    maintainer_email='youremail@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_node      = control.nodes.JoystickNode:main',
            'esp_bridge_node    = control.nodes.ESPBridgeNode:main',
            'esp_actuator_node  = control.nodes.ESPActuatorNode:main',
            'odom_publisher_node = control.nodes.OdomPublisherNode:main',
            'logger_node = control.nodes.LoggerNode:main',
            'health_monitor_node = control.nodes.HealthMonitorNode:main',
            'mock_sensor_node = control.nodes.MockSensorNode:main',
            'manual_navigation_node = control.nodes.ManualNavigationNode:main',
            'autonomous_navigation_node = control.nodes.AutonomousNavigationNode:main',
            'mission_manager_node = control.nodes.MissionManagerNode:main',
            'hoverboard_node      = control.nodes.HoverBoardNode:main',
            "camera_control_server = control.API.ControlServer:main",
        ],
    },
)
