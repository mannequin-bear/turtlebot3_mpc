from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot3_mpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'nav_msgs',
        'casadi',
        'numpy'
        
    ],
    zip_safe=True,
    maintainer='manish',
    maintainer_email='manish@todo.todo',
    description='My Model Predictive Control Implementation',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mpc = turtlebot3_mpc.turtlebot3_mpc:main',
            'bot_mpc = turtlebot3_mpc.bot_mpc:main'
        ],
    },
)
