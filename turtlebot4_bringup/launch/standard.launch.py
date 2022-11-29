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

<<<<<<< HEAD
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
=======
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import LaunchConfigurationNotEquals
>>>>>>> Use PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import PushRosNamespace

from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot Namespace')
]


def generate_launch_description():
    lc = LaunchContext()
    ld = LaunchDescription()

    diagnostics_enable = EnvironmentVariable('TURTLEBOT4_DIAGNOSTICS', default_value='1')

    pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_bringup')
    pkg_turtlebot4_diagnostics = get_package_share_directory('turtlebot4_diagnostics')
    pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')

    param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_bringup, 'config', 'turtlebot4.yaml']),
        description='Turtlebot4 Robot param file'
    )

    param_file = LaunchConfiguration('param_file')
    namespace = LaunchConfiguration('namespace')

    namespaced_param_file = RewrittenYaml(
        source_file=param_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True)

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
    description_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_description, 'launch', 'robot_description.launch.py']
    )

    actions = [
            PushRosNamespace(
                condition=LaunchConfigurationNotEquals('namespace', ''),
                namespace=namespace),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([turtlebot4_robot_launch_file]),
                launch_arguments=[('model', 'standard'),
                                ('param_file', namespaced_param_file)]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([joy_teleop_launch_file])),


            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([rplidar_launch_file])),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([oakd_launch_file]),
                launch_arguments=[('tf_prefix', 'oakd_pro')]),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([description_launch_file]),
                launch_arguments=[('model', 'standard')]),
        ]

    if (diagnostics_enable.perform(lc)) == '1':
        actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource([diagnostics_launch_file])))
    
    turtlebot4_standard = GroupAction(actions)

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(param_file_cmd)
    ld.add_action(turtlebot4_standard)
    return ld
