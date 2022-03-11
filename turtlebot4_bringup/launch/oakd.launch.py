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

from launch import launch_description_sources, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import launch_ros.descriptions


def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name',   default='oak')

    mode = LaunchConfiguration('mode', default='depth')
    lrcheck = LaunchConfiguration('lrcheck', default=True)
    extended = LaunchConfiguration('extended', default=False)
    subpixel = LaunchConfiguration('subpixel', default=True)
    confidence = LaunchConfiguration('confidence', default=200)
    LRchecktresh = LaunchConfiguration('LRchecktresh', default=5)

    robot_model = DeclareLaunchArgument('model',
                                        default_value='standard',
                                        choices=['standard', 'lite'],
                                        description='Turtlebot4 Model')

    declare_camera_name_cmd = DeclareLaunchArgument(
        'camera_name',
        default_value=camera_name,
        description='The name of the camera. It can be different from the camera model \
                     and it will be used in naming TF.')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value=mode,
        description='set to depth or disparity. \
                     Setting to depth will publish depth or else will publish disparity.')

    declare_lrcheck_cmd = DeclareLaunchArgument(
        'lrcheck',
        default_value=lrcheck)

    declare_extended_cmd = DeclareLaunchArgument(
        'extended',
        default_value=extended)

    declare_subpixel_cmd = DeclareLaunchArgument(
        'subpixel',
        default_value=subpixel)

    declare_confidence_cmd = DeclareLaunchArgument(
        'confidence',
        default_value=confidence)

    declare_LRchecktresh_cmd = DeclareLaunchArgument(
        'LRchecktresh',
        default_value=LRchecktresh)

    stereo_node = Node(
            package='depthai_examples',
            executable='stereo_node',
            output='screen',
            parameters=[{'camera_name': camera_name},
                        {'mode': mode},
                        {'lrcheck': lrcheck},
                        {'extended': extended},
                        {'subpixel': subpixel}])

    oakd_left_standard_stf = Node(
            name='oakd_standard_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['-0.0596', '0', '0.17933', '0', '0', '0', 'base_link', 'oak_left_camera_optical_frame'],
            condition=LaunchConfigurationEquals('model', 'standard')
        )

    oakd_right_standard_stf = Node(
            name='oakd_standard_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['-0.0596', '0', '0.17933', '0', '0', '0', 'base_link', 'oak_right_camera_optical_frame'],
            condition=LaunchConfigurationEquals('model', 'standard')
        )

    oakd_rgb_standard_stf = Node(
            name='oakd_standard_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['-0.0596', '0', '0.17933', '0', '0', '0', 'base_link', 'oak_rgb_camera_optical_frame'],
            condition=LaunchConfigurationEquals('model', 'standard')
        )

    oakd_lite_stf = Node(
            name='oakd_lite_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['-0.09', '0', '0.18588622', '0', '0', '0', 'base_link', 'oak-d-base-frame'],
            condition=LaunchConfigurationEquals('model', 'lite')
        )

    ld = LaunchDescription()
    ld.add_action(robot_model)
    ld.add_action(declare_camera_name_cmd)

    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_lrcheck_cmd)
    ld.add_action(declare_extended_cmd)
    ld.add_action(declare_subpixel_cmd)
    ld.add_action(declare_confidence_cmd)
    ld.add_action(declare_LRchecktresh_cmd)

    ld.add_action(stereo_node)
    ld.add_action(oakd_left_standard_stf)
    ld.add_action(oakd_right_standard_stf)
    ld.add_action(oakd_rgb_standard_stf)
    ld.add_action(oakd_lite_stf)

    return ld
