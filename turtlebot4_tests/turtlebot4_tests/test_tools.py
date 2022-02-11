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

import sys
import time
import os 

class TestResult:
    # Test results: PASSED, FAILED, WARNING, N/A
    def __init__(self, name, result):
        self.name = name
        self.result = result

class Test:
    def __init__(self, name, func):
        self.name = name
        self.func = func

class Tester():
    tests = []

    def addTest(self, name, func):
        self.tests.append(Test(name, func))

    def runTest(self, index):
        test = self.tests[index]

        # Format
        dash = '-' * 30
        print(dash)
        print('{:^30s}'.format('Running Test: ' + test.name))
        print(dash)

        status = False

        # Enables automatic rerunning of test if test execution failed
        try:
            while not status:
                status = test.func()
        # ctrl+c to exit test
        except KeyboardInterrupt:
            pass

    def showTestOptions(self):
        # Format
        dash = '-' * 30
        print(dash)
        print('{:^30s}'.format('Test Options'))
        print(dash)

        for i, test in enumerate(self.tests):
            print('{:d}. {}'.format(i + 1, test.name))
        print('{:d}. All Tests'.format(len(self.tests) + 1))
        print('{:d}. Exit'.format(len(self.tests) + 2))
        try:
            option = int(input('Select an option: '))
        except ValueError:
            print('Invalid input')
            self.showTestOptions()

        # Exit option
        if option == len(self.tests) + 2:
            sys.exit()
        # All tests option
        elif option == len(self.tests) + 1:
            for index in range(len(self.tests)):
                self.runTest(index)
        elif option > 0 and option <= len(self.tests):
            self.runTest(option - 1)
        else:
            print('Invalid input')

        time.sleep(1)
        self.showTestOptions()

    def run(self):
        self.showTestOptions()

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

