from setuptools import setup
import os
from glob import glob

package_name = 'turtle_multi_target'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Multi-target turtle follower with tf2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_tf2_broadcaster = turtle_multi_target.turtle_tf2_broadcaster:main',
            'target_switcher = turtle_multi_target.target_switcher:main',
            'turtle_controller = turtle_multi_target.turtle_controller:main',
            'keyboard_switch = turtle_multi_target.keyboard_switch:main',
        ],
    },
)
