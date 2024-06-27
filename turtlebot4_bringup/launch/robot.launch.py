#!/usr/bin/env python3
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model')
]


def generate_launch_description():

    pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_bringup')
    create3_republisher = get_package_share_directory('create3_republisher')

    param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_bringup, 'config', 'turtlebot4.yaml']),
        description='Turtlebot4 Robot param file'
    )

    create3_param_file_cmd = DeclareLaunchArgument(
        'create3_param_file',
        default_value=PathJoinSubstitution(
            [create3_republisher, 'bringup', 'params.yaml']),
        description='Create3 republisher param file'
    )

    turtlebot4_param_yaml_file = LaunchConfiguration('param_file')
    create3_repub_param_yaml_file = LaunchConfiguration('create3_param_file')

    turtlebot4_node = Node(
        package='turtlebot4_node',
        executable='turtlebot4_node',
        parameters=[turtlebot4_param_yaml_file,
                    {'model': LaunchConfiguration('model')}],
        output='screen')

    turtlebot4_base_node = Node(
        package='turtlebot4_base',
        executable='turtlebot4_base_node',
        parameters=[turtlebot4_param_yaml_file],
        output='screen',
        condition=LaunchConfigurationEquals('model', 'standard')
    )
    create3_republisher_node = Node(
        package='create3_republisher',
        executable='create3_republisher',
        parameters=[create3_repub_param_yaml_file,
                    {'robot_namespace': '_do_not_use'}],
        output='screen',
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(param_file_cmd)
    ld.add_action(create3_param_file_cmd)
    ld.add_action(turtlebot4_node)
    ld.add_action(turtlebot4_base_node)
    if (os.environ.get('ROS_DISCOVERY_SERVER', '').strip(' ;\"')):
        ld.add_action(create3_republisher_node)
    return ld
