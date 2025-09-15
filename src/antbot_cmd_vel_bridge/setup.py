import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'antbot_cmd_vel_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),          
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cg',
    maintainer_email='natthawejumjai@gmail.com',
    description='Bridge node: subscribe Twist or TwistStamped and republish TwistStamped to controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = antbot_cmd_vel_bridge.bridge_node:main',
        ],
    },
)
