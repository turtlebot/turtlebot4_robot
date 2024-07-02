from glob import glob

import os

from setuptools import setup

package_name = 'turtlebot4_diagnostics'

setup(
    name=package_name,
    version='1.0.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rkreinin',
    maintainer_email='rkreinin@clearpathrobotics.com',
    description='Turtlebot4 Diagnostics',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diagnostics_updater = turtlebot4_diagnostics.diagnostics_updater:main'
        ],
    },
)
