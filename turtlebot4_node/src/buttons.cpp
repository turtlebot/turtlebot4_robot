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

#include "turtlebot4_node/buttons.hpp"
#include <iostream>
#include <vector>
#include <memory>
#include <utility>

using turtlebot4::Buttons;

Buttons::Buttons(
  Turtlebot4Model model,
  std::vector<Turtlebot4Button> buttons,
  std::shared_ptr<rclcpp::Node> & nh,
  std::shared_ptr<GpioInterface> gpio_interface, bool use_sim)
: user_button_pins_{HMI_BUTTON_1_PIN,
    HMI_BUTTON_2_PIN,
    HMI_BUTTON_3_PIN,
    HMI_BUTTON_4_PIN},
  buttons_(buttons),
  gpio_interface_(gpio_interface),
  use_sim_(use_sim),
  nh_(nh),
  model_(model)
{
  RCLCPP_INFO(nh_->get_logger(), "Buttons Init");

  create3_buttons_sub_ = nh_->create_subscription<irobot_create_msgs::msg::InterfaceButtons>(
    "interface_buttons",
    rclcpp::SensorDataQoS(),
    std::bind(&Buttons::create3_buttons_callback, this, std::placeholders::_1));

  if (model_ == Turtlebot4Model::STANDARD) {
    hmi_buttons_pub_ =
      nh_->create_publisher<turtlebot4_msgs::msg::UserButton>(
      "hmi/buttons",
      rclcpp::QoS(
        rclcpp::KeepLast(
          10)));
    if (use_sim) {
      hmi_buttons_sub_ = nh_->create_subscription<std_msgs::msg::Int32>(
        "hmi/buttons/_set",
        rclcpp::SensorDataQoS(),
        std::bind(&Buttons::hmi_buttons_callback, this, std::placeholders::_1));
    } else {
      for (auto i = 0; i < HMI_BUTTON_COUNT; i++) {
        gpio_interface->add_line(user_button_pins_[i], LINE_DIRECTION_INPUT);
      }
    }
  }
}

/**
 * @brief Poll buttons, call callbacks if pressed
 */
void Buttons::spin_once()
{
  if (!use_sim_ && model_ == Turtlebot4Model::STANDARD) {
    for (auto i = 0; i < HMI_BUTTON_COUNT; i++) {
      uint8_t read_val = !gpio_interface_->read(user_button_pins_[i]);
      buttons_.at(HMI_1 + i).set_state(static_cast<Turtlebot4ButtonState>(read_val));
    }
    publish_hmi_buttons();
  }

  // Spin buttons
  for (size_t i = 0; i < buttons_.size(); i++) {
    buttons_.at(i).spin_once();
  }
}

void Buttons::hmi_buttons_callback(const std_msgs::msg::Int32::SharedPtr hmi_buttons_msg)
{
  if (hmi_buttons_msg->data == 0) {
    for (int i = 0; i < HMI_BUTTON_COUNT; i++) {
      buttons_.at(HMI_1 + i).set_state(RELEASED);
    }
  } else if (hmi_buttons_msg->data > 0 && hmi_buttons_msg->data <= HMI_BUTTON_COUNT) {
    buttons_.at(HMI_1 + hmi_buttons_msg->data - 1).set_state(PRESSED);
  } else {
    RCLCPP_ERROR(nh_->get_logger(), "Invalid HMI Button id %d", hmi_buttons_msg->data);
  }
  publish_hmi_buttons();
}

void Buttons::create3_buttons_callback(
  const irobot_create_msgs::msg::InterfaceButtons::SharedPtr create3_buttons_msg)
{
  buttons_.at(CREATE3_1).set_state(
    static_cast<Turtlebot4ButtonState>(create3_buttons_msg->button_1.
    is_pressed));
  buttons_.at(CREATE3_POWER).set_state(
    static_cast<Turtlebot4ButtonState>(create3_buttons_msg->
    button_power.is_pressed));
  buttons_.at(CREATE3_2).set_state(
    static_cast<Turtlebot4ButtonState>(create3_buttons_msg->button_2.
    is_pressed));
}

void Buttons::publish_hmi_buttons()
{
  turtlebot4_msgs::msg::UserButton msg;
  for (int i = 0; i < HMI_BUTTON_COUNT; i++) {
    if (buttons_.at(HMI_1 + i).next_state_ == Turtlebot4ButtonState::RELEASED) {
      msg.button[i] = false;
    } else {
      msg.button[i] = true;
    }
  }
  hmi_buttons_pub_->publish(std::move(msg));
}
