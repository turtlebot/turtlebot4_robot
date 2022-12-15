/*
 * Copyright 2021 Clearpath Robotics, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#ifndef TURTLEBOT4_BASE__TURTLEBOT4_HPP_
#define TURTLEBOT4_BASE__TURTLEBOT4_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>

#include "turtlebot4_base/display.hpp"
#include "turtlebot4_base/buttons.hpp"
#include "turtlebot4_base/leds.hpp"
#include "turtlebot4_base/gpio_interface.hpp"
#include "turtlebot4_base/i2c_interface.hpp"

#include "turtlebot4_msgs/msg/user_button.hpp"
#include "turtlebot4_msgs/msg/user_led.hpp"
#include "turtlebot4_msgs/msg/user_display.hpp"


namespace turtlebot4_base
{

class Turtlebot4 : public rclcpp::Node
{
public:
  // Constructor and Destructor
  Turtlebot4();
  virtual ~Turtlebot4() {}

private:
  void run();

  // Run buttons timer
  void buttons_timer(const std::chrono::milliseconds timeout);

  // Node
  rclcpp::Node::SharedPtr node_handle_;

  // Interfaces
  std::shared_ptr<GpioInterface> gpiochip0_;
  std::shared_ptr<I2cInterface> i2c3_;

  // Display
  std::unique_ptr<Display> display_;

  // Buttons
  std::unique_ptr<Buttons> buttons_;

  // Leds
  std::unique_ptr<Leds> leds_;

  // Timer
  rclcpp::TimerBase::SharedPtr buttons_timer_;
};

}  // namespace turtlebot4_base

#endif  // TURTLEBOT4_BASE__TURTLEBOT4_HPP_
