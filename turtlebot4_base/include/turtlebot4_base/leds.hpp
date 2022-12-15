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

#ifndef TURTLEBOT4_BASE__LEDS_HPP_
#define TURTLEBOT4_BASE__LEDS_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"

#include "turtlebot4_msgs/msg/user_led.hpp"
#include "turtlebot4_base/gpio_interface.hpp"
#include "turtlebot4_node/utils.hpp"

namespace turtlebot4_base
{

using turtlebot4::Turtlebot4LedType;
using turtlebot4::Turtlebot4LedColor;
using turtlebot4::Turtlebot4LedEnum;

struct Turtlebot4Led
{
  Turtlebot4LedType type_;
  std::shared_ptr<GpioInterface> gpio_interface_;
  uint8_t green_pin_, red_pin_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr led_sub_;

  // Green Only constructor
  Turtlebot4Led(std::shared_ptr<GpioInterface> gpio_interface, uint8_t green_pin)
  : type_(Turtlebot4LedType::GREEN_ONLY),
    gpio_interface_(gpio_interface),
    green_pin_(green_pin)
  {
    gpio_interface->add_line(green_pin, LINE_DIRECTION_OUTPUT);
  }

  // Red Green constructor
  Turtlebot4Led(
    std::shared_ptr<GpioInterface> gpio_interface,
    uint8_t green_pin, uint8_t red_pin)
  : type_(Turtlebot4LedType::RED_GREEN),
    gpio_interface_(gpio_interface),
    green_pin_(green_pin),
    red_pin_(red_pin)
  {
    gpio_interface->add_line(green_pin, LINE_DIRECTION_OUTPUT);
    gpio_interface->add_line(red_pin, LINE_DIRECTION_OUTPUT);
  }

  void create_subscription(rclcpp::Node::SharedPtr nh, std::string topic)
  {
    led_sub_ = nh->create_subscription<std_msgs::msg::Int32>(
      topic, rclcpp::QoS(
        rclcpp::KeepLast(
          10)), std::bind(&Turtlebot4Led::led_callback, this, std::placeholders::_1));
  }

  void led_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    switch (msg->data) {
      case Turtlebot4LedColor::OFF:
        {
          gpio_interface_->write(green_pin_, 0);
          if (type_ == Turtlebot4LedType::RED_GREEN) {
            gpio_interface_->write(red_pin_, 0);
          }
          break;
        }

      case Turtlebot4LedColor::GREEN:
        {
          gpio_interface_->write(green_pin_, 1);
          if (type_ == Turtlebot4LedType::RED_GREEN) {
            gpio_interface_->write(red_pin_, 0);
          }
          break;
        }

      case Turtlebot4LedColor::RED:
        {
          gpio_interface_->write(green_pin_, 0);
          if (type_ == Turtlebot4LedType::RED_GREEN) {
            gpio_interface_->write(red_pin_, 1);
          }
          break;
        }

      case Turtlebot4LedColor::YELLOW:
        {
          if (type_ == Turtlebot4LedType::RED_GREEN) {
            gpio_interface_->write(green_pin_, 1);
            gpio_interface_->write(red_pin_, 1);
          }
          break;
        }

      default:
        {
          return;
        }
    }
  }
};

class Leds
{
public:
  Leds(
    std::shared_ptr<rclcpp::Node> & nh, std::shared_ptr<GpioInterface> gpio_interface);

private:
  std::shared_ptr<rclcpp::Node> nh_;

  rclcpp::Subscription<turtlebot4_msgs::msg::UserLed>::SharedPtr user_led_sub_;

  std::map<Turtlebot4LedEnum, std::shared_ptr<Turtlebot4Led>> leds_;

  std::shared_ptr<GpioInterface> gpio_interface_;
};

}  // namespace turtlebot4_base

#endif  // TURTLEBOT4_BASE__LEDS_HPP_
