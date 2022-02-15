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

#include "turtlebot4_node/display.hpp"

#include <string>
#include <vector>
#include <memory>
#include <utility>

#include "turtlebot4_node/gpio_interface.hpp"

using turtlebot4::Display;
using turtlebot4::Turtlebot4MenuEntry;

/**
 * @brief Display constructor
 * @input nh - Turtlebot4 Node Handle
 */
Display::Display(
  std::vector<Turtlebot4MenuEntry> entries,
  std::shared_ptr<rclcpp::Node> & nh, std::shared_ptr<I2cInterface> i2c,
  std::shared_ptr<GpioInterface> gpio, bool use_sim)
: nh_(nh),
  menu_entries_(entries),
  scroll_position_(0),
  selected_line_(0),
  ip_(UNKNOWN_IP),
  battery_percentage_(0),
  oled_(i2c, 0x3C),
  use_sim_(use_sim)
{
  RCLCPP_INFO(nh_->get_logger(), "Init Display");

  display_raw_pub_ =
    nh_->create_publisher<std_msgs::msg::String>(
    "hmi/display/_raw",
    rclcpp::QoS(rclcpp::KeepLast(10)));
  display_selected_pub_ =
    nh_->create_publisher<std_msgs::msg::Int32>(
    "hmi/display/_selected",
    rclcpp::QoS(
      rclcpp::KeepLast(
        10)));
  display_set_sub_ = nh_->create_subscription<std_msgs::msg::String>(
    "hmi/display/set",
    rclcpp::SensorDataQoS(),
    std::bind(&Display::display_callback, this, std::placeholders::_1));

  if (!use_sim) {
    // Initialize display reset pin and pull it low
    gpio->add_line(HMI_DISPLAY_RESET_PIN, LINE_DIRECTION_OUTPUT);
    // Set reset pin high to enter normal operation mode
    gpio->write(HMI_DISPLAY_RESET_PIN, 1);
    // Initialize oled driver
    oled_.Init();
  }

  // Initialize menu entries
  set_menu_entries();
  visible_entries_ = get_visible_entries();
}

/**
 * @brief Set IP address
 * @input ip - IP address as std::string
 */
void Display::set_ip(std::string ip)
{
  ip_ = ip;
}

/**
 * @brief Set battery percentage
 * @input battery_state_msg - Battery state message from Create3
 */
void Display::set_battery(const sensor_msgs::msg::BatteryState::SharedPtr & battery_state_msg)
{
  battery_percentage_ = static_cast<int>(battery_state_msg->percentage * 100);
}

void Display::scroll_down()
{
  if (menu_override_) {
    return;
  }
  // Last possible scroll position, last line selected
  if (static_cast<size_t>(scroll_position_ + SSD1306_NUM_LINES) == menu_entries_.size() &&
    selected_line_ == SSD1306_NUM_LINES - 1)
  {
    return;
  }

  if (selected_line_ == SSD1306_NUM_LINES - 1) {
    if (menu_entries_.size() > static_cast<size_t>(scroll_position_ + SSD1306_NUM_LINES)) {
      scroll_position_++;
    }
  } else {
    selected_line_++;
  }
}

void Display::scroll_up()
{
  if (menu_override_) {
    return;
  }
  // First scroll position, first line selected
  if (scroll_position_ == 0 && selected_line_ == 0) {
    return;
  }

  if (selected_line_ == 0) {
    scroll_position_--;
  } else {
    selected_line_--;
  }
}

void Display::select()
{
  if (menu_override_) {
    return;
  }

  if (visible_entries_[selected_line_].cb_ != nullptr) {
    visible_entries_[selected_line_].cb_();
  }
}

void Display::back()
{
  if (menu_override_) {
    menu_override_ = false;
  } else {
    scroll_position_ = 0;
    selected_line_ = 0;
  }
}

void Display::update_header()
{
  header_ = ip_ + " " + std::to_string(battery_percentage_) + "%";

  // Pad string
  if (header_.length() < SSD1306_CHAR_PER_LINE_HEADER) {
    header_.insert(header_.length(), SSD1306_CHAR_PER_LINE_HEADER - header_.length(), ' ');
  } else if (header_.length() > SSD1306_CHAR_PER_LINE_HEADER) {
    // Remove excess characters
    header_ = header_.substr(0, SSD1306_CHAR_PER_LINE_HEADER);
  }
}

/**
 * @brief Format and return default display message
 */
void Display::set_menu_entries()
{
  for (auto & entry : menu_entries_) {
    pad_line(entry.name_);
  }
}

void Display::pad_line(std::string & line)
{
  // Pad string
  if (line.length() < SSD1306_CHAR_PER_LINE) {
    line.insert(line.length(), SSD1306_CHAR_PER_LINE - line.length(), ' ');
  } else if (line.length() > SSD1306_CHAR_PER_LINE) {
    // Remove excess characters
    line = line.substr(0, SSD1306_CHAR_PER_LINE);
  }
}

void Display::draw_header()
{
  // Draw header
  oled_.SetCursor(0, 0);
  oled_.WriteString(ip_, SSD1306_HEADER_FONT, White);
  oled_.DrawBattery(100, 0, White);
  oled_.WriteString(std::to_string(battery_percentage_), SSD1306_HEADER_FONT, White);
  oled_.Line(
    0, SSD1306_HEADER_FONT_HEIGHT + 1, SSD1306_WIDTH, SSD1306_HEADER_FONT_HEIGHT + 1,
    White);
}

std::vector<Turtlebot4MenuEntry> Display::get_visible_entries()
{
  std::vector<Turtlebot4MenuEntry>::const_iterator first = menu_entries_.begin() + scroll_position_;
  std::vector<Turtlebot4MenuEntry>::const_iterator last;

  if (menu_entries_.size() > static_cast<size_t>(scroll_position_ + SSD1306_NUM_LINES)) {
    last = menu_entries_.begin() + scroll_position_ + SSD1306_NUM_LINES;
  } else {
    last = menu_entries_.end();
  }
  return std::vector<Turtlebot4MenuEntry>(first, last);
}

void Display::show_message(std::vector<std::string> message)
{
  std::vector<std::string>::const_iterator first = message.begin();
  std::vector<std::string>::const_iterator last;

  if (message.size() > static_cast<size_t>(SSD1306_NUM_LINES)) {
    last = message.begin() + SSD1306_NUM_LINES;
  } else {
    last = message.end();
  }
  display_lines_ = std::vector<std::string>(first, last);

  for (auto & line : display_lines_) {
    pad_line(line);
  }

  menu_override_ = true;
}

void Display::show_message(std::string message)
{
  display_lines_ = std::vector<std::string>(SSD1306_NUM_LINES);

  for (int i = 0; i < SSD1306_NUM_LINES; i++) {
    if (message.length() < static_cast<size_t>(SSD1306_CHAR_PER_LINE * i)) {
      display_lines_[i] = "";
    } else if (message.length() < static_cast<size_t>(SSD1306_CHAR_PER_LINE * (i + 1))) {
      display_lines_[i] =
        message.substr(SSD1306_CHAR_PER_LINE * i, message.length() - (SSD1306_CHAR_PER_LINE * i));
    } else {
      display_lines_[i] = message.substr(SSD1306_CHAR_PER_LINE * i, SSD1306_CHAR_PER_LINE);
    }
  }

  for (auto & line : display_lines_) {
    pad_line(line);
  }

  menu_override_ = true;
}

/**
 * @brief Update display
 */
void Display::update()
{
  update_header();
  visible_entries_ = get_visible_entries();

  // Write to display
  if (!use_sim_) {
    oled_.Fill(Black);
    // Draw header
    draw_header();

    if (menu_override_) {
      // Draw lines
      for (size_t i = 0; i < display_lines_.size(); i++) {
        oled_.SetCursor(0, ((i + 1) * SSD1306_FONT_HEIGHT) + 2);
        oled_.WriteString(display_lines_.at(i), SSD1306_FONT, White);
      }
    } else {
      // Draw lines
      for (size_t i = 0; i < visible_entries_.size(); i++) {
        oled_.SetCursor(0, ((i + 1) * SSD1306_FONT_HEIGHT) + 2);
        if (i == selected_line_) {
          oled_.Line(
            0, oled_.GetCursor().CurrentY - 1, visible_entries_.at(
              i).name_.length() * SSD1306_FONT_WIDTH - 1, oled_.GetCursor().CurrentY - 1, White);
          oled_.WriteString(visible_entries_.at(i).name_, SSD1306_FONT, Black);
        } else {
          oled_.WriteString(visible_entries_.at(i).name_, SSD1306_FONT, White);
        }
      }
    }

    oled_.UpdateScreen();
  }

  // Publish display as a single string
  auto display_raw_msg = std::make_unique<std_msgs::msg::String>();
  // Publish currently selected message
  auto display_selected_msg = std::make_unique<std_msgs::msg::Int32>();

  if (menu_override_) {
    display_selected_msg->data = -1;
  } else {
    display_selected_msg->data = selected_line_;
  }

  display_raw_msg->data = header_;

  if (menu_override_) {
    for (size_t i = 0; i < display_lines_.size(); i++) {
      display_raw_msg->data += display_lines_.at(i);
    }
  } else {
    for (size_t i = 0; i < visible_entries_.size(); i++) {
      display_raw_msg->data += visible_entries_.at(i).name_;
    }
  }

  display_raw_pub_->publish(std::move(display_raw_msg));
  display_selected_pub_->publish(std::move(display_selected_msg));
}

/**
 * @brief Spin Once
 */
void Display::spin_once()
{
  update();
}

void Display::display_callback(const std_msgs::msg::String::SharedPtr display_msg)
{
  show_message(display_msg->data);
}
