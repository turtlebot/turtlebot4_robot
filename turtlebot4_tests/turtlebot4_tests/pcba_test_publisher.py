#!/usr/bin/env python3

# Copyright 2024 Clearpath Robotics, Inc.
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
# @author Hilary Luo (hluo@clearpathrobotics.com)

# Description:
#   This script allows the testing of the PCBA to ensure functionality
#   without the presence of a create 3. This is possible by publishing to
#   topics otherwise published by the create 3.
#
# Expected Operation:
#   All lights should be green except for the battery light (which will alternate
#   between green and blinking red), user led 1 (which will alternate between green
#   and off) and user led 2 (which will cycle between green, red and off).
#
# Usage:
#   The pi must be powered with all connections present to the pcba. The pcba
#   must also be powered directly to replace the create3 power.
#
#   ros2 run turtlebot4_tests pcba_test_publisher
#
#   or with namespacing:
#   ros2 run turtlebot4_tests pcba_test_publisher --ros-args -r __ns:=/my_robot

from irobot_create_msgs.msg import WheelStatus

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState

from turtlebot4_msgs.msg import UserLed


class PcbaTestPublisher(Node):

    def __init__(self):
        super().__init__('pcba_test_publisher')

        # Publishers
        self.battery_publisher_ = self.create_publisher(BatteryState, 'battery_state', 10)
        self.wheel_status_publisher_ = self.create_publisher(WheelStatus, 'wheel_status', 10)
        self.led_publisher_ = self.create_publisher(UserLed, 'hmi/led', 10)

        # Timer
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Battery values
        self.battery_min = 5.0
        self.battery_max = 25.0
        self.battery_step = 5.0
        self.battery_percentage = self.battery_min
        self.increasing = True

        # LED values
        self.leds = [UserLed.USER_LED_1, UserLed.USER_LED_2]
        self.colors = [UserLed.COLOR_OFF, UserLed.COLOR_GREEN, UserLed.COLOR_RED]
        self.current_color_index = 0
        self.blink_period = 1000
        self.duty_cycle = 1.0

    def timer_callback(self):
        self.get_logger().info('Overriding battery state, user leds and wheel status')

        # Publish BatteryState
        battery_msg = BatteryState()
        battery_msg.percentage = self.battery_percentage / 100.0
        self.battery_publisher_.publish(battery_msg)

        if self.increasing:
            self.battery_percentage += self.battery_step
            if self.battery_percentage >= self.battery_max:
                self.increasing = False
        else:
            self.battery_percentage -= self.battery_step
            if self.battery_percentage <= self.battery_min:
                self.increasing = True

        # Publish WheelStatus
        wheel_status_msg = WheelStatus()
        wheel_status_msg.wheels_enabled = True
        self.wheel_status_publisher_.publish(wheel_status_msg)

        # Publish UserLed
        for led in self.leds:
            led_msg = UserLed()
            led_msg.led = led
            led_msg.color = self.colors[self.current_color_index]
            # Note that User LED 1 will go green in both cases due to hardware limitations
            led_msg.blink_period = self.blink_period
            led_msg.duty_cycle = self.duty_cycle
            self.led_publisher_.publish(led_msg)

        self.current_color_index = (self.current_color_index + 1) % len(self.colors)


def main(args=None):
    rclpy.init(args=args)
    pcba_test_publisher = PcbaTestPublisher()
    rclpy.spin(pcba_test_publisher)
    pcba_test_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
