from setuptools import find_packages, setup
from glob import glob

package_name = 'rover26_autonomy'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover26',
    maintainer_email='rorzblaban@todo.todo',
    description='Rover26 autonomy: lane following (PID) + Nav2 obstacle avoidance',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Lane detection
            'lane_detection          = rover26_autonomy.nodes.lane_detection_node:main',

            # Detection → SLAM map bridge (lanes, potholes)
            'generic_points_publisher = rover26_autonomy.nodes.generic_points_publisher_node:main',

            # Pothole detection
            'pothole_detection       = rover26_autonomy.nodes.pothole_detection_node:main',
            'pothole_publisher       = rover26_autonomy.nodes.pothole_publisher_node:main',

            # Lane path publisher
            'lane_path_publisher     = rover26_autonomy.nodes.generic_points_publisher_node:main',

            # Lane goal publisher
            'lane_goal_publisher     = rover26_autonomy.nodes.lane_goal_publisher:main',
            'simple_lane_goal        = rover26_autonomy.nodes.simple_lane_goal:main',

            # Utilities
            'odom_tf_broadcaster     = rover26_autonomy.nodes.odom_tf_broadcaster:main',
            'twist_unstamper         = rover26_autonomy.nodes.twist_unstamper:main',

            #real tests
            'real_lidar_relay           = rover26_autonomy.nodes.real_lidar_relay:main',   
        ],
    },
)