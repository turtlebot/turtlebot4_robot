from setuptools import setup

package_name = 'turtlebot4_tests'

setup(
    name=package_name,
    version='1.0.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rkreinin',
    maintainer_email='rkreinin@clearpathrobotics.com',
    description='Turtlebot4 System Tests',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_tests = turtlebot4_tests.ros_tests:main',
        ],
    },
)
