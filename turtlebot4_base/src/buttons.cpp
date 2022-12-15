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

#include "turtlebot4_base/buttons.hpp"
#include <iostream>
#include <vector>
#include <memory>
#include <utility>

using turtlebot4_base::Buttons;

Buttons::Buttons(
  std::shared_ptr<rclcpp::Node> & nh,
  std::shared_ptr<GpioInterface> gpio_interface)
: gpio_interface_(gpio_interface),
  nh_(nh)
{
  hmi_buttons_pub_ =
    nh_->create_publisher<turtlebot4_msgs::msg::UserButton>(
    "hmi/buttons",
    rclcpp::SensorDataQoS());

  nh_->declare_parameter("gpio.user_button_1", 13);
  nh_->declare_parameter("gpio.user_button_2", 19);
  nh_->declare_parameter("gpio.user_button_3", 16);
  nh_->declare_parameter("gpio.user_button_4", 26);

  user_button_gpio_[0] = nh_->get_parameter("gpio.user_button_1").as_int();
  user_button_gpio_[1] = nh_->get_parameter("gpio.user_button_2").as_int();
  user_button_gpio_[2] = nh_->get_parameter("gpio.user_button_3").as_int();
  user_button_gpio_[3] = nh_->get_parameter("gpio.user_button_4").as_int();

  for (auto i = 0; i < HMI_BUTTON_COUNT; i++) {
    gpio_interface->add_line(user_button_gpio_[i], LINE_DIRECTION_INPUT);
  }
}

/**
 * @brief Poll buttons
 */
void Buttons::spin_once()
{
  auto button_msg = std::make_unique<turtlebot4_msgs::msg::UserButton>();

  for (auto i = 0; i < HMI_BUTTON_COUNT; i++) {
    uint8_t read_val = gpio_interface_->read(user_button_gpio_[i]);
    button_msg->button[i] = !read_val;
  }

  hmi_buttons_pub_->publish(std::move(button_msg));
}
