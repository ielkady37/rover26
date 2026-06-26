from setuptools import find_packages, setup

package_name = 'cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mansour',
    maintainer_email='ahmedmonsour5@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_streaming_node = cv.nodes.CameraStreamingNode:main',
            'camera_viewer_node = cv.nodes.CameraViewerNode:main',
            'face_recognition_node = cv.nodes.FaceRecognitionNode:main',
            'pothole_detection_node = cv.nodes.PotholesDetectionNode:main',
            'lane_detection_node = cv.nodes.LaneDetectionNode:main',
        ],
    },
)
