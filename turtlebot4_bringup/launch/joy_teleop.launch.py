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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_bringup')

    controller_config_cmd = DeclareLaunchArgument(
        'controller_config',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_bringup, 'config', 'turtlebot4_controller.config.yaml']),
        description='Turtlebot4 Joy teleop param file'
    )

    namespace = LaunchConfiguration('namespace')

    controller_config = RewrittenYaml(
        source_file=LaunchConfiguration('controller_config'),
        root_key=namespace,
        param_rewrites={},
        convert_types=True)

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux_node',
        remappings=[('/diagnostics', 'diagnostics')]
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[controller_config]
    )

    ld = LaunchDescription()
    ld.add_action(controller_config_cmd)
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)

    return ld
