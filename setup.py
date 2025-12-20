from setuptools import setup
import os
from glob import glob

package_name = 'autonomous_navigator'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('autonomous_navigator/launch/*launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('autonomous_navigator/config/*.yaml')),
        (os.path.join('share', package_name, 'config'),
            glob('autonomous_navigator/config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Autonomous navigation package for TurtleBot3 using Nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_navigator = autonomous_navigator.waypoint_navigator:main',
            'goal_publisher = autonomous_navigator.goal_publisher:main',
        ],
    },
)