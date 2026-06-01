from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'scout_rl'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='RL training package for Scout Mini',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_td3 = scout_rl.train_td3:main',
            'test_policy = scout_rl.test_policy:main',
        ],
    },
)