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

import math
import os
import subprocess
import time

import psutil


class TestResult:
    # Test results: PASSED, FAILED, WARNING, N/A
    def __init__(self, name, result):
        self.name = name
        self.result = result


class Test:
    count = 0

    def __init__(self, name, func):
        self.name = name
        self.func = func


class Tester():
    tests = []
    processes = []
    rosbag_count = 0

    def __init__(self, results_dir):
        self.results_dir = results_dir

    def addTest(self, name, func):
        self.tests.append(Test(name, func))

    def runTest(self, index):
        test = self.tests[index]
        self.tests[index].count += 1

        # Format
        dash = '-' * 30
        print(dash)
        print('{:^30s}'.format('Running Test: ' + test.name))
        print(dash)

        status = False

        self.rosbag_record(index)
        # Enables automatic rerunning of test if test execution failed
        try:
            while not status:
                status = test.func()
        # ctrl+c to exit test
        except KeyboardInterrupt:
            pass

        self.rosbag_stop()

    def showTestOptions(self):
        # Format
        dash = '-' * 30
        print(dash)
        print('{:^30s}'.format('Test Options'))
        print(dash)

        for i, test in enumerate(self.tests):
            print('{:d}. {}'.format(i + 1, test.name))
        print('{:d}. Exit'.format(len(self.tests) + 1))
        try:
            option = int(input('Select an option: '))
        except ValueError:
            print('Invalid input')
            self.showTestOptions()

        # Exit option
        if option == len(self.tests) + 1:
            return
        elif option > 0 and option <= len(self.tests):
            self.runTest(option - 1)
        else:
            print('Invalid input')

        time.sleep(1)
        self.showTestOptions()

    def run(self):
        self.showTestOptions()

    def rosbag_record(self, index):
        self.rosbag_process = subprocess.Popen(
            'exec ' + 'ros2 bag record -o {0}/rosbag2/{1}-{2} -a'.format(
                self.results_dir, self.tests[index].name.replace(' ', '_'),
                self.tests[index].count),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            shell=True)

    def rosbag_stop(self):
        parent = psutil.Process(self.rosbag_process.pid)
        for child in parent.children(recursive=True):
            child.kill()
        parent.kill()


# Displays user prompt message, then requests user input of (y/n).
# Returns TestResult with 'PASSED' or 'FAILED' results corresponding to user input.
def userInputTestResults(prompt, test_case):
    print(prompt)
    response = input('y/n: ')

    if response == 'y' or response == 'Y':
        return TestResult(test_case, 'PASSED')
    elif response == 'n' or response == 'N':
        return TestResult(test_case, 'FAILED')
    else:
        print('Invalid input')
        return userInputTestResults(prompt, test_case)


# Returns TestResult with 'PASSED' or 'FAILED' results based on value of boolean input
def boolTestResults(bool_input, test_case):
    result = ''

    if bool_input:
        result = 'PASSED'
    else:
        result = 'FAILED'

    return TestResult(test_case, result)


# Returns TestResults with 'N/A' for test runs that failed to execute
def notApplicableTestResult(test_case):
    result = 'N/A'
    message = '%s failed to execute' % test_case

    return TestResult(message, result)


def printTestResults(name, results):
    # Format
    dash = '-' * 30
    print('\033[0;37;40m')
    print(dash)
    print('{:^30s}'.format(name + ' Results'))
    print(dash)

    for test in results:
        name_string = '\033[0;37;40m' + test.name
        if test.result == 'PASSED':
            result_string = '\033[1;32;40m PASSED'
        elif test.result == 'FAILED':
            result_string = '\033[1;31;40m FAILED'
        elif test.result == 'WARNING':
            result_string = '\033[1;33;40m WARNING'
        elif test.result == 'N/A':
            result_string = '\033[1;34;40m N/A'
        else:
            print('Invalid test result for {}'.format(name_string))
            continue
        print('{:<30s}{:>20s}\033[0;39;49m'.format(name_string, result_string))


def logTestResults(file, name, results):
    os.makedirs(os.path.dirname(file), exist_ok=True)
    f = open(file, 'a')
    f.write('--------------------------------------\n')
    header = '{}\n'.format(name)
    f.write(header)

    for test in results:
        output_result = '{}: {}\n'.format(test.name, test.result)
        f.write(output_result)

    f.close()


def euler_from_quaternion(quaternion):
    """
    Convert a quaternion (w in last place) to euler roll, pitch, yaw.

    quaternion = [x, y, z, w]
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
