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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument(
        'tf_prefix',
        default_value='oakd_pro',
        description='The name of the camera. \
                     It can be different from the camera model \
                     and it will be used in naming TF.'),
    DeclareLaunchArgument(
        'publish_urdf',
        default_value='False',
        description='Whether to publish the urdf'),
    DeclareLaunchArgument(
        'colorResolution',
        choices=['1080p', '4K'],
        default_value='1080p',
        description='The resolution of the color camera'),
    DeclareLaunchArgument(
        'useVideo',
        default_value='False',
        description='Whether to publish a video of color image'),
    DeclareLaunchArgument(
        'usePreview',
        default_value='True',
        description='Whether to publish a preview of color image'),
    DeclareLaunchArgument(
        'useDepth',
        default_value='True',
        description='Whether to publish the depth image'),
    DeclareLaunchArgument(
        'previewWidth',
        default_value='250',
        description='Width of preview image'),
    DeclareLaunchArgument(
        'previewHeight',
        default_value='250',
        description='Height of preview image')
]


def generate_launch_description():
    pkg_depthai_examples = get_package_share_directory('depthai_examples')

    rgb_stereo_launch_file = PathJoinSubstitution(
        [pkg_depthai_examples, 'launch', 'rgb_stereo_node.launch.py'])

    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rgb_stereo_launch_file]),
        launch_arguments={'colorResolution': LaunchConfiguration('colorResolution'),
                          'useVideo': LaunchConfiguration('useVideo'),
                          'usePreview': LaunchConfiguration('usePreview'),
                          'useDepth': LaunchConfiguration('useDepth'),
                          'previewWidth': LaunchConfiguration('previewWidth'),
                          'previewHeight': LaunchConfiguration('previewHeight'),
                          'publish_urdf': LaunchConfiguration('publish_urdf'),
                          'tf_prefix': LaunchConfiguration('tf_prefix')}.items())

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(oakd_launch)
    return ld
