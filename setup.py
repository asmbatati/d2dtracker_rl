import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'd2dtracker_rl'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config', 'mavros'), glob('config/mavros/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdulrahman S. Al-Batati',
    maintainer_email='asmalbatati@hotmail.com',
    description='Reinforcement learning for drone-to-drone interception (Gymnasium + SB3).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train = d2dtracker_rl.train:main',
            'evaluate = d2dtracker_rl.evaluate:main',
            'curriculum = d2dtracker_rl.curriculum:main',
            'train_intercept = d2dtracker_rl.aerogym_intercept.train_intercept:main',
            'evaluate_intercept = d2dtracker_rl.aerogym_intercept.evaluate_intercept:main',
        ],
    },
)
