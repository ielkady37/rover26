from setuptools import setup
import os
from glob import glob

package_name = 'rover_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alkady',
    maintainer_email='youremail@example.com',
    description='Navigation stack for rover26',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
