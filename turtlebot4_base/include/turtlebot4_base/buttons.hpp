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

#ifndef TURTLEBOT4_BASE__BUTTONS_HPP_
#define TURTLEBOT4_BASE__BUTTONS_HPP_

#include <string>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "turtlebot4_msgs/msg/user_button.hpp"
#include "turtlebot4_base/gpio_interface.hpp"
#include "turtlebot4_node/utils.hpp"

namespace turtlebot4_base
{

using turtlebot4::HMI_BUTTON_COUNT;

class Buttons
{
public:
  Buttons(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<GpioInterface> gpio_interface);

  void spin_once();

private:
  uint8_t user_button_gpio_[HMI_BUTTON_COUNT];

  std::shared_ptr<GpioInterface> gpio_interface_;

  std::shared_ptr<rclcpp::Node> nh_;
  rclcpp::Publisher<turtlebot4_msgs::msg::UserButton>::SharedPtr hmi_buttons_pub_;
};

}  // namespace turtlebot4_base

#endif  // TURTLEBOT4_BASE__BUTTONS_HPP_
