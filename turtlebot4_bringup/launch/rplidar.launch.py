# Copyright 2021 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
]


def generate_launch_description():

    rplidar_standard_stf = Node(
            name='rplidar_standard_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['-0.04816', '0', '0.14', '1.5707', '0.0', '0.0', 'base_link', 'laser'],
            condition=LaunchConfigurationEquals('model', 'standard')
        )

    rplidar_lite_stf = Node(
            name='rplidar_lite_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.07', '1.5707', '0.0', '0.0', 'base_link', 'laser'],
            condition=LaunchConfigurationEquals('model', 'lite')
        )

    rplidar_node = Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/RPLIDAR',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rplidar_lite_stf)
    ld.add_action(rplidar_standard_stf)
    ld.add_action(rplidar_node)
