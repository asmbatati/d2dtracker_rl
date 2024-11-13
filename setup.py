
from setuptools import setup
import os
from glob import glob

package_name = 'd2dtracker_rl'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Reinforcement learning for D2DTracker drone simulations using ROS 2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_d2d_node = d2dtracker_rl.train_d2d_node:main',
            'd2d_rl_render_node = d2dtracker_rl.d2d_rl_render_node:main',
            'rl_node = d2dtracker_rl.d2dtracker_training:main',
            'test_gym = d2dtracker_rl.test_gym:main',
        ],
    },
)
