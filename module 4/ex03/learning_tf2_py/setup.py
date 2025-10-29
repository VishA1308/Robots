from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'learning_tf2_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Learning tf2 with time travel example',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broadcaster = learning_tf2_py.broadcaster:main',
            'listener = learning_tf2_py.listener:main',
        ],
    },
)