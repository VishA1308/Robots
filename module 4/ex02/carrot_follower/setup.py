from setuptools import setup
import os
from glob import glob

package_name = 'carrot_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Carrot follower demo with tf2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_tf2_broadcaster = carrot_follower.turtle_tf2_broadcaster:main',
            'turtle_tf2_listener = carrot_follower.turtle_tf2_listener:main',
            'carrot_broadcaster = carrot_follower.carrot_broadcaster:main',
            'spawn_turtle = carrot_follower.spawn_turtle:main',
        ],
    },
)
