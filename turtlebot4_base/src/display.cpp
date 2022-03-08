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

#include "turtlebot4_base/display.hpp"

#include <string>
#include <vector>
#include <memory>
#include <utility>

#include "turtlebot4_base/gpio_interface.hpp"

using turtlebot4_base::Display;

/**
 * @brief Display constructor
 * @input nh - Turtlebot4 Node Handle
 */
Display::Display(
  std::shared_ptr<rclcpp::Node> & nh,
  std::shared_ptr<I2cInterface> i2c,
  std::shared_ptr<GpioInterface> gpio)
: nh_(nh),
  oled_(i2c, 0x3C)
{
  RCLCPP_INFO(nh_->get_logger(), "Init Display");

  display_sub_ = nh_->create_subscription<turtlebot4_msgs::msg::UserDisplay>(
    "hmi/display",
    rclcpp::SensorDataQoS(),
    std::bind(&Display::display_callback, this, std::placeholders::_1));

  nh_->declare_parameter("gpio.display_reset", 2);
  // Initialize display reset pin and pull it low
  gpio->add_line(nh_->get_parameter("gpio.display_reset").as_int(), LINE_DIRECTION_OUTPUT);
  // Set reset pin high to enter normal operation mode
  gpio->write(nh_->get_parameter("gpio.display_reset").as_int(), 1);
  // Initialize oled driver
  oled_.Init();
}

void Display::display_callback(const turtlebot4_msgs::msg::UserDisplay::SharedPtr msg)
{
  oled_.Fill(Black);
  // Draw header
  oled_.SetCursor(0, 0);
  oled_.WriteString(msg->ip, SSD1306_HEADER_FONT, White);
  oled_.DrawBattery(100, 0, White);
  oled_.WriteString(msg->battery, SSD1306_HEADER_FONT, White);
  oled_.Line(
    0, SSD1306_HEADER_FONT_HEIGHT + 1,
    SSD1306_WIDTH, SSD1306_HEADER_FONT_HEIGHT + 1,
    White);

  // Draw entries
  for (size_t i = 0; i < msg->entries.size(); i++) {
    oled_.SetCursor(0, ((i + 1) * SSD1306_FONT_HEIGHT) + 2);
    // Highlight selected entry
    if (i == static_cast<size_t>(msg->selected_entry)) {
      oled_.Line(
        0, oled_.GetCursor().CurrentY - 1,
        msg->entries[i].length() * SSD1306_FONT_WIDTH - 1, oled_.GetCursor().CurrentY - 1, White);
      oled_.WriteString(msg->entries[i], SSD1306_FONT, Black);
    } else {
      oled_.WriteString(msg->entries[i], SSD1306_FONT, White);
    }
  }
  oled_.UpdateScreen();
}
