#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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


from diagnostic_msgs.msg import DiagnosticStatus

from diagnostic_updater import FrequencyStatusParam, HeaderlessTopicDiagnostic, Updater

from irobot_create_msgs.msg import DockStatus, HazardDetectionVector, Mouse, WheelStatus

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import BatteryState, Image, Imu, LaserScan


hazards_type = {0: 'BACKUP_LIMIT',
                1: 'BUMP',
                2: 'CLIFF',
                3: 'STALL',
                4: 'WHEEL_DROP',
                5: 'OBJECT_PROXIMITY'}


class Turtlebot4DiagnosticUpdater(Node):

    def __init__(self):
        super().__init__('turtlebot4_diagnostics')

        # Create updater
        self.updater = Updater(self)
        self.updater.setHardwareID('Turtlebot4')

        # Subscribe to topics
        self.battery_sub = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_status_callback,
            qos_profile_sensor_data
        )

        self.wheel_sub = self.create_subscription(
            WheelStatus,
            'wheel_status',
            self.wheel_status_callback,
            qos_profile_sensor_data
        )

        self.dock_sub = self.create_subscription(
            DockStatus,
            'dock_status',
            self.dock_callback,
            qos_profile_sensor_data
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            qos_profile_sensor_data
        )

        self.color_image_sub = self.create_subscription(
            Image,
            'oakd/rgb/preview/image_raw',
            self.color_image_callback,
            qos_profile_sensor_data
        )

        self.hazard_detection_sub = self.create_subscription(
            HazardDetectionVector,
            'hazard_detection',
            self.hazard_detection_callback,
            qos_profile_sensor_data
        )

        self.imu_sub = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.imu_sub = self.create_subscription(
            Mouse,
            'mouse',
            self.mouse_callback,
            qos_profile_sensor_data
        )

        # Initial values
        self.battery_percentage = 0.0
        self.battery_voltage = 0.0
        self.wheels_enabled = False
        self.is_docked = False
        self.dock_visible = False
        self.lidar_freq_bounds = {'min': 5, 'max': 10}
        self.camera_freq_bounds = {'min': 5, 'max': 60}
        self.imu_freq_bounds = {'min': 100, 'max': 100}
        self.mouse_freq_bounds = {'min': 61, 'max': 63}
        self.detections = []

        # Add Diagnostic status
        self.updater.add('Battery Percentage', self.check_battery_percentage)
        self.updater.add('Battery Voltage', self.check_battery_voltage)
        self.updater.add('Wheel Status', self.check_wheel_status)
        self.updater.add('Dock Status', self.check_dock_status)
        self.updater.add('Hazard Detections', self.check_hazard_detections)

        # Add Frequency status
        self.lidar_freq = HeaderlessTopicDiagnostic('/scan',
                                                    self.updater,
                                                    FrequencyStatusParam(
                                                        self.lidar_freq_bounds, 0.1, 10))

        self.color_image_freq = HeaderlessTopicDiagnostic('/color/preview/image',
                                                          self.updater,
                                                          FrequencyStatusParam(
                                                              self.camera_freq_bounds, 0.1, 10))

        self.imu_freq = HeaderlessTopicDiagnostic('/imu',
                                                  self.updater,
                                                  FrequencyStatusParam(
                                                      self.imu_freq_bounds, 0.1, 10))

        self.mouse_freq = HeaderlessTopicDiagnostic('/mouse',
                                                    self.updater,
                                                    FrequencyStatusParam(
                                                        self.mouse_freq_bounds, 0.1, 10))

        self.updater.force_update()

    def check_battery_percentage(self, stat):
        if self.battery_percentage > 0.2:
            stat.summary(DiagnosticStatus.OK, 'OK')
        elif self.battery_percentage > 0.05:
            stat.summary(DiagnosticStatus.WARN, 'Low')
        else:
            stat.summary(DiagnosticStatus.ERROR, 'Critical')
        stat.add('Battery Percentage', '%.2f' % self.battery_percentage)
        return stat

    def check_battery_voltage(self, stat):
        if self.battery_voltage > 15.0:
            stat.summary(DiagnosticStatus.OK, 'OK')
        elif self.battery_voltage > 13.0:
            stat.summary(DiagnosticStatus.WARN, 'Low')
        else:
            stat.summary(DiagnosticStatus.ERROR, 'Critical')
        stat.add('Battery Voltage', '%.2f' % self.battery_voltage)
        return stat

    def check_wheel_status(self, stat):
        if self.wheels_enabled:
            stat.summary(DiagnosticStatus.OK, 'Enabled')
        else:
            stat.summary(DiagnosticStatus.WARN, 'Disabled')
        stat.add('Wheel Status', '%r' % self.wheels_enabled)
        return stat

    def check_dock_status(self, stat):
        if self.is_docked:
            stat.summary(DiagnosticStatus.OK, 'Docked')
        # Not docked
        else:
            # Check battery percentage
            if self.battery_percentage > 0.2:
                stat.summary(DiagnosticStatus.OK, 'Undocked')
            elif self.battery_percentage > 0.05:
                stat.summary(DiagnosticStatus.WARN, 'Undocked, consider docking soon')
            else:
                stat.summary(DiagnosticStatus.ERROR, 'Battery critical, please dock the robot')
        stat.add('Docked Status', '%r' % self.is_docked)
        return stat

    def hazards_to_str(self):
        hazards_str = []
        for hazard in self.detections:
            hazards_str.append(hazards_type[hazard])
        return hazards_str

    def check_hazard_detections(self, stat):
        if len(self.detections) > 0:
            stat.summary(DiagnosticStatus.WARN, 'Hazard(s) detected')
        else:
            stat.summary(DiagnosticStatus.OK, 'No hazards detected')
        stat.add('Hazard Detection', '%r' % self.hazards_to_str())
        return stat

    def battery_status_callback(self, msg):
        self.battery_percentage = msg.percentage
        self.battery_voltage = msg.voltage

    def wheel_status_callback(self, msg):
        self.wheels_enabled = msg.wheels_enabled

    def dock_callback(self, msg):
        self.is_docked = msg.is_docked
        self.dock_visible = msg.dock_visible

    def hazard_detection_callback(self, msg: HazardDetectionVector):
        self.detections = []
        for hazard in msg.detections:
            self.detections.append(hazard.type)

    def lidar_callback(self, msg):
        self.lidar_freq.tick()

    def color_image_callback(self, msg):
        self.color_image_freq.tick()

    def imu_callback(self, msg):
        self.imu_freq.tick()

    def mouse_callback(self, msg):
        self.mouse_freq.tick()


def main(args=None):
    rclpy.init(args=args)

    node = Turtlebot4DiagnosticUpdater()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
