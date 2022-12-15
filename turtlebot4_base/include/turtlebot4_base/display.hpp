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

#ifndef TURTLEBOT4_BASE__DISPLAY_HPP_
#define TURTLEBOT4_BASE__DISPLAY_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "turtlebot4_base/ssd1306.hpp"
#include "turtlebot4_base/buttons.hpp"
#include "turtlebot4_msgs/msg/user_display.hpp"


namespace turtlebot4_base
{

class Display
{
public:
  // Constructor and Destructor
  Display(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<I2cInterface> i2c,
    std::shared_ptr<GpioInterface> gpio);
  virtual ~Display() {}

private:
  void display_callback(const turtlebot4_msgs::msg::UserDisplay::SharedPtr msg);

  // Node handle
  std::shared_ptr<rclcpp::Node> nh_;
  rclcpp::Subscription<turtlebot4_msgs::msg::UserDisplay>::SharedPtr display_sub_;

  // Display driver
  Ssd1306 oled_;
};

}  // namespace turtlebot4_base

#endif  // TURTLEBOT4_BASE__DISPLAY_HPP_
