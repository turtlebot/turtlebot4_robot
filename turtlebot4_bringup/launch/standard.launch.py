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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.path_join_substitution import PathJoinSubstitution


def generate_launch_description():

    pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_bringup')
    pkg_turtlebot4_diagnostics = get_package_share_directory('turtlebot4_diagnostics')

    # Launch files
    turtlebot4_robot_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'robot.launch.py'])
    joy_teleop_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'joy_teleop.launch.py'])
    diagnostics_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_diagnostics, 'launch', 'diagnostics.launch.py'])
    rplidar_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'rplidar.launch.py'])
    oakd_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'oakd.launch.py'])

    standard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot4_robot_launch_file]),
        launch_arguments=[('model', 'standard')])
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([joy_teleop_launch_file]))
    diagnostics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([diagnostics_launch_file]))
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rplidar_launch_file]))
    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([oakd_launch_file]))

    ld = LaunchDescription()
    ld.add_action(standard_launch)
    ld.add_action(teleop_launch)
    ld.add_action(diagnostics_launch)
    ld.add_action(rplidar_launch)
    ld.add_action(oakd_launch)
    return ld
