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

# Description:
#   This script exposes a set of useful tests at the ROS-level,
#   via ROS topics, to verify the functionality of core features.
#   In addition, these tests together serve as a useful robot-level
#   diagnostic tool, be identifying the root cause of problems,
#   or at the very least, narrowing down on where the root cause(s) may be.
#
# Usage:
#   ros2 run turtlebot4_tests ros_tests

import math
import os
from os.path import expanduser
import threading
import time

from irobot_create_msgs.action import Dock, DriveDistance, RotateAngle, Undock
from irobot_create_msgs.msg import DockStatus, InterfaceButtons, LightringLeds

from nav_msgs.msg import Odometry

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String

from turtlebot4_msgs.msg import UserButton, UserLed

from turtlebot4_tests.test_tools import (boolTestResults, euler_from_quaternion,
                                         logTestResults, notApplicableTestResult,
                                         printTestResults, Tester, userInputTestResults)


class Turtlebot4RosTests(Node):

    def __init__(self):
        super().__init__('turtlebot4_ros_tests')

        self.results_dir = expanduser('~') + \
            '/turtlebot4_test_results/' + \
            time.strftime('%Y_%m_%d-%H_%M_%S')

        self.log_file_name = self.results_dir + '/turtlebot4_test_log.txt'

        os.makedirs(os.path.dirname(self.log_file_name), exist_ok=True)

        print('Saving results to ' + self.results_dir)

        self.tester = Tester(self.results_dir)

        # Add tests
        self.tester.addTest('Light Ring Test', self.lightRingTest)
        self.tester.addTest('Create3 Button Test', self.createButtonTest)
        self.tester.addTest('Drive Test', self.driveTest)
        self.tester.addTest('Dock Test', self.dockTest)
        self.tester.addTest('User LED Test', self.userLedTest)
        self.tester.addTest('User Display Test', self.displayTest)
        self.tester.addTest('User Button Test', self.userButtonTest)
        self.tester.addTest('TurtleBot 4 Lite Tests', self.liteTests)
        self.tester.addTest('TurtleBot 4 Tests', self.standardTests)

    def lightRingTest(self):
        results = []
        # Delay between publishing messages and requesting user input
        response_delay = 1
        print('Testing Create3 Light Ring... \n')

        pub = self.create_publisher(LightringLeds, '/cmd_lightring', qos_profile_sensor_data)
        time.sleep(response_delay)

        msg = LightringLeds()
        msg.override_system = True

        msg.header.stamp = self.get_clock().now().to_msg()
        pub.publish(msg)

        # Set all leds to red
        for led in msg.leds:
            led.red = 255
            led.green = 0
            led.blue = 0

        msg.header.stamp = self.get_clock().now().to_msg()
        pub.publish(msg)

        time.sleep(response_delay)
        results.append(userInputTestResults('Did all lights turn red?', 'All Red'))

        # Set all leds to green
        for led in msg.leds:
            led.red = 0
            led.green = 255
            led.blue = 0

        msg.header.stamp = self.get_clock().now().to_msg()
        pub.publish(msg)

        time.sleep(response_delay)
        results.append(userInputTestResults('Did all lights turn green?', 'All Green'))

        # Set all lights to blue
        for led in msg.leds:
            led.red = 0
            led.green = 0
            led.blue = 255

        msg.header.stamp = self.get_clock().now().to_msg()
        pub.publish(msg)

        time.sleep(response_delay)
        results.append(userInputTestResults('Did all lights turn blue?', 'All Blue'))

        # Set all lights to white
        for led in msg.leds:
            led.red = 255
            led.green = 255
            led.blue = 255

        msg.header.stamp = self.get_clock().now().to_msg()
        pub.publish(msg)

        time.sleep(response_delay)
        results.append(userInputTestResults('Did all lights turn white?', 'All White'))

        # Set all leds to off
        for led in msg.leds:
            led.red = 0
            led.green = 0
            led.blue = 0

        msg.header.stamp = self.get_clock().now().to_msg()
        pub.publish(msg)

        time.sleep(response_delay)
        results.append(userInputTestResults('Did all lights turn off?', 'All Off'))

        msg.override_system = False
        msg.header.stamp = self.get_clock().now().to_msg()
        pub.publish(msg)

        self.destroy_publisher(pub)

        printTestResults('Lighting Test', results)
        logTestResults(self.log_file_name, 'Lighting Test', results)
        return True

    def createButtonCallback(self, msg):
        self.create_button_msg = msg

    def createButtonTest(self):
        results = []
        # Delay between publishing messages and requesting user input
        response_delay = 0.5
        print('Testing Create3 Buttons... \n')

        self.create_button_msg = InterfaceButtons()

        create_button_sub = self.create_subscription(InterfaceButtons,
                                                     '/interface_buttons',
                                                     self.createButtonCallback,
                                                     qos_profile_sensor_data)
        time.sleep(response_delay)

        print('Press Create3 Button 1')
        while not self.create_button_msg.button_1.is_pressed:
            pass
        results.append(boolTestResults(True, 'Create Button 1'))

        print('Press Create3 Button Power')
        while not self.create_button_msg.button_power.is_pressed:
            pass
        results.append(boolTestResults(True, 'Create Button Power'))

        print('Press Create3 Button 2')
        while not self.create_button_msg.button_2.is_pressed:
            pass
        results.append(boolTestResults(True, 'Create Button 2'))

        self.destroy_subscription(create_button_sub)
        printTestResults('Create3 Button Test', results)
        logTestResults(self.log_file_name, 'Create3 Button Test', results)
        return True

    def userLedTest(self):
        results = []
        # Delay between publishing messages and requesting user input
        response_delay = 1
        print('Testing User LEDs... \n')

        user_led_pub = self.create_publisher(UserLed, '/hmi/led', qos_profile_sensor_data)
        time.sleep(response_delay)

        msg = UserLed()

        # Start with both User LEDs Off
        msg.led = 0
        msg.color = 0
        user_led_pub.publish(msg)
        msg.led = 1
        user_led_pub.publish(msg)

        msg.led = 0
        msg.color = 1
        msg.blink_period = 1000
        msg.duty_cycle = 1.0

        user_led_pub.publish(msg)
        time.sleep(response_delay)
        results.append(userInputTestResults('Is User LED 1 Green?', 'User LED 1 Green'))
        time.sleep(response_delay)

        msg.led = 1
        msg.color = 1

        user_led_pub.publish(msg)
        time.sleep(response_delay)
        results.append(userInputTestResults('Is User LED 2 Green?', 'User LED 2 Green'))
        time.sleep(response_delay)

        msg.color = 2

        user_led_pub.publish(msg)
        time.sleep(response_delay)
        results.append(userInputTestResults('Is User LED 2 Red?', 'User LED 2 Red'))
        time.sleep(response_delay)

        msg.color = 3

        user_led_pub.publish(msg)
        time.sleep(response_delay)
        results.append(userInputTestResults('Is User LED 2 Yellow?', 'User LED 2 Yellow'))
        time.sleep(response_delay)

        msg.led = 0
        msg.color = 1
        msg.duty_cycle = 0.5

        user_led_pub.publish(msg)
        msg.led = 1
        user_led_pub.publish(msg)
        time.sleep(response_delay)
        results.append(userInputTestResults('Are both User LEDs blinking?', 'User LED Blinking'))

        msg.led = 0
        msg.color = 0
        user_led_pub.publish(msg)
        msg.led = 1
        user_led_pub.publish(msg)

        self.destroy_publisher(user_led_pub)

        printTestResults('HMI Test', results)
        logTestResults(self.log_file_name, 'Lighting Test', results)
        return True

    def displayTest(self):
        results = []
        # Delay between publishing messages and requesting user input
        response_delay = 1
        print('Testing the display... \n')

        display_pub = self.create_publisher(
                        String,
                        '/hmi/display/message',
                        qos_profile_sensor_data)
        time.sleep(response_delay)

        msg = String()

        msg.data = 'Turtlebot4 Display Test'

        display_pub.publish(msg)
        time.sleep(response_delay)

        results.append(userInputTestResults("Does the display show 'Turtlebot4 Display Test'?",
                                            'Display Message'))

        self.destroy_publisher(display_pub)

        printTestResults('User Display Test', results)
        logTestResults(self.log_file_name, 'User Display Test', results)
        return True

    def userButtonCallback(self, msg):
        self.button_msg = msg

    def userButtonTest(self):
        results = []
        # Delay between publishing messages and requesting user input
        response_delay = 0.5
        print('Testing User Buttons... \n')

        self.button_msg = UserButton()

        user_button_sub = self.create_subscription(UserButton,
                                                   '/hmi/buttons',
                                                   self.userButtonCallback,
                                                   qos_profile_sensor_data)
        time.sleep(response_delay)

        for i in range(0, 4):
            print('Press User Button ' + str(i + 1))
            # Wait for button press
            while not self.button_msg.button[i]:
                pass

            results.append(boolTestResults(True, 'User Button %c' % str(i + 1)))
            time.sleep(response_delay)

        self.destroy_subscription(user_button_sub)
        printTestResults('User Button Test', results)
        logTestResults(self.log_file_name, 'User Button Test', results)
        return True

    def dockCallback(self, msg):
        self.is_docked = msg.is_docked

    def dockTest(self):
        results = []
        self.is_docked = False

        dock_sub = self.create_subscription(DockStatus,
                                            '/dock_status',
                                            self.dockCallback,
                                            qos_profile_sensor_data)
        undock_action_client = ActionClient(self, Undock, '/undock')
        dock_action_client = ActionClient(self, Dock, '/dock')
        undock_goal_msg = Undock.Goal()
        dock_goal_msg = Dock.Goal()

        time.sleep(1)

        if not self.is_docked:
            print('Place the robot on the dock')

        # Wait for robot to be docked
        while not self.is_docked:
            pass

        time.sleep(3)

        print('Undocking...')

        undock_action_client.wait_for_server()
        undock_goal_result = undock_action_client.send_goal(undock_goal_msg)

        results.append(boolTestResults(not undock_goal_result.result.is_docked, 'Undocking'))

        # Undocking failed
        if undock_goal_result.result.is_docked:
            print('Undocking failed, skipping docking')
            results.append(notApplicableTestResult('Docking'))
        else:
            time.sleep(2)
            print('Docking...')
            dock_action_client.wait_for_server()
            goal_result = dock_action_client.send_goal(dock_goal_msg)
            results.append(boolTestResults(goal_result.result.is_docked, 'Docking'))

        printTestResults('Dock Test', results)
        logTestResults(self.log_file_name, 'Dock Test', results)

        self.destroy_subscription(dock_sub)
        return True

    def odomCallback(self, msg):
        self.odom = msg
        self.odom_received = True

    def driveTest(self):
        results = []
        poses = []
        self.odom_received = False
        self.odom = Odometry()

        drive_distance = 0.25
        rotate_angle = math.pi/2

        drive_action_client = ActionClient(self, DriveDistance, '/drive_distance')
        rotate_action_client = ActionClient(self, RotateAngle, '/rotate_angle')
        drive_goal_msg = DriveDistance.Goal()
        drive_goal_msg.distance = drive_distance
        rotate_goal_msg = RotateAngle.Goal()
        rotate_goal_msg.angle = rotate_angle

        dock_sub = self.create_subscription(Odometry,
                                            '/odom',
                                            self.odomCallback,
                                            qos_profile_sensor_data)

        print('The robot will drive forwards 0.25m then turn 90 degrees 4 times.')
        input('Press enter to start.')

        if not self.odom_received:
            print('Waiting for odometry...')

            while(not self.odom_received):
                time.sleep(0.1)

            print('Odometry received')

        poses.append(self.odom.pose.pose)

        for i in range(0, 4):
            drive_action_client.wait_for_server()
            drive_goal_result = drive_action_client.send_goal(drive_goal_msg)
            poses.append(drive_goal_result.result.pose.pose)

            time.sleep(1)

            rotate_action_client.wait_for_server()
            rotate_goal_result = rotate_action_client.send_goal(rotate_goal_msg)
            poses.append(rotate_goal_result.result.pose.pose)

            time.sleep(1)

        delta_positions = []
        delta_yaw = []
        for i in range(0, len(poses) - 1):
            delta_positions.append(math.dist([poses[i].position.x, poses[i].position.y],
                                             [poses[i+1].position.x, poses[i+1].position.y]))
            r1, p1, y1 = euler_from_quaternion(poses[i].orientation)
            r2, p2, y2 = euler_from_quaternion(poses[i+1].orientation)
            delta_yaw.append(abs((math.degrees(y1) - math.degrees(y2) + 180) % 360 - 180))

        print('{:<8} | {:<20} | {:<20}'.format('Action', 'Δ Position (m)', 'Δ Yaw (deg)'))

        for i in range(0, len(delta_positions)):
            print('{:<8} | {:<20} | {:<20}'.format(i + 1, delta_positions[i], delta_yaw[i]))
            if i % 2 == 0:
                results.append(boolTestResults(
                    math.isclose(delta_positions[i], drive_distance, rel_tol=0.1),
                    'Action {0} Drive'.format(i+1)))
            else:
                results.append(boolTestResults(
                    math.isclose(delta_yaw[i], rotate_angle, rel_tol=1),
                    'Action {0} Rotate'.format(i+1)))

        printTestResults('Drive Test', results)
        logTestResults(self.log_file_name, 'Drive Test', results)

        self.destroy_client(drive_action_client)
        self.destroy_client(rotate_action_client)
        self.destroy_subscription(dock_sub)
        return True

    def liteTests(self):
        for i in range(0, 4):
            self.tester.runTest(i)
        return True

    def standardTests(self):
        for i in range(0, 7):
            self.tester.runTest(i)
        return True


def main(args=None):
    rclpy.init(args=args)

    tests = Turtlebot4RosTests()

    thread = threading.Thread(target=rclpy.spin, args=(tests,), daemon=True)
    thread.start()

    print('Running Turtlebot4 ROS tests...\n')

    try:
        tests.tester.run()
    except KeyboardInterrupt:
        pass

    tests.destroy_node()
    rclpy.shutdown()

    thread.join()


if __name__ == '__main__':
    main()
