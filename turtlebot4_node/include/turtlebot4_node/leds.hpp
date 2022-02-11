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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include "turtlebot4_msgs/msg/user_led.hpp"
#include "turtlebot4_node/gpio_interface.hpp"
#include "turtlebot4_node/utils.hpp"

#include "std_msgs/msg/int32.hpp"

namespace turtlebot4
{

enum Turtlebot4LedEnum
{
  POWER,
  MOTORS,
  COMMS,
  WIFI,
  BATTERY,
  USER_1,
  USER_2,
  COUNT
};

enum Turtlebot4LedType
{
  GREEN_ONLY,
  RED_GREEN
};

enum Turtlebot4LedColor
{
  OFF = 0,
  GREEN = 1,
  RED = 2,
  YELLOW = 3
};

struct Turtlebot4Led
{
  Turtlebot4LedType type_;
  bool use_sim_;
  std::shared_ptr<GpioInterface> gpio_interface_;
  uint8_t green_pin_, red_pin_;
  uint32_t on_period_ms_, off_period_ms_;
  std::chrono::time_point<std::chrono::steady_clock> last_on_time_, last_off_time_;
  Turtlebot4LedColor current_color_, next_color_, blink_color_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr led_pub_;

  Turtlebot4Led(bool use_sim, std::shared_ptr<GpioInterface> gpio_interface, uint8_t green_pin) : type_(Turtlebot4LedType::GREEN_ONLY),
                                                                                    use_sim_(use_sim),
                                                                                    gpio_interface_(gpio_interface),
                                                                                    green_pin_(green_pin),
                                                                                    on_period_ms_(0), 
                                                                                    off_period_ms_(1000),
                                                                                    current_color_(Turtlebot4LedColor::OFF),
                                                                                    next_color_(Turtlebot4LedColor::OFF),
                                                                                    blink_color_(Turtlebot4LedColor::OFF)
  {
    if (!use_sim)
    {
      gpio_interface->add_line(green_pin, LINE_DIRECTION_OUTPUT);
    }
  }

  Turtlebot4Led(bool use_sim, std::shared_ptr<GpioInterface> gpio_interface, 
                uint8_t green_pin, uint8_t red_pin) : type_(Turtlebot4LedType::RED_GREEN),
                                                      use_sim_(use_sim),
                                                      gpio_interface_(gpio_interface),
                                                      green_pin_(green_pin),
                                                      red_pin_(red_pin),
                                                      on_period_ms_(0), 
                                                      off_period_ms_(1000),
                                                      current_color_(Turtlebot4LedColor::OFF),
                                                      next_color_(Turtlebot4LedColor::OFF),
                                                      blink_color_(Turtlebot4LedColor::OFF)
  {
    if (!use_sim)
    {
      gpio_interface->add_line(green_pin, LINE_DIRECTION_OUTPUT);
      gpio_interface->add_line(red_pin, LINE_DIRECTION_OUTPUT);  
    }
  }

  void create_publisher(rclcpp::Node::SharedPtr nh, std::string topic)
  {
    led_pub_ = nh->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(rclcpp::KeepLast(10)));
  }
  
  void write_led(Turtlebot4LedColor color)
  {
    if (use_sim_)
    {
      return;
    }
    
    switch(color)
    {
      case Turtlebot4LedColor::OFF:
      {
        gpio_interface_->write(green_pin_, 0);
        if (type_ == Turtlebot4LedType::RED_GREEN)
        {
          gpio_interface_->write(red_pin_, 0);
        }
        break;
      }

      case Turtlebot4LedColor::GREEN:
      {
        gpio_interface_->write(green_pin_, 1);
        if (type_ == Turtlebot4LedType::RED_GREEN)
        {
          gpio_interface_->write(red_pin_, 0);
        }
        break;
      }

      case Turtlebot4LedColor::RED:
      {
        gpio_interface_->write(green_pin_, 0);
        if (type_ == Turtlebot4LedType::RED_GREEN)
        {
          gpio_interface_->write(red_pin_, 1);
        }
        break;
      }

      case Turtlebot4LedColor::YELLOW:
      {
        if (type_ == Turtlebot4LedType::RED_GREEN)
        {
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

  void spin_once()
  {
    switch(current_color_)
    {
      case Turtlebot4LedColor::OFF:
      {
        // Duty cycle > 0
        if (on_period_ms_ > 0)
        {
          // Time to turn on
          if (std::chrono::steady_clock::now() > last_off_time_ + std::chrono::milliseconds(off_period_ms_))
          {
            write_led(blink_color_);
            last_on_time_ = std::chrono::steady_clock::now();
            current_color_ = blink_color_;
          }
        }
        break;
      }

      case Turtlebot4LedColor::GREEN:
      case Turtlebot4LedColor::RED:
      case Turtlebot4LedColor::YELLOW:
      {
        // Duty cycle < 1.0 or blink color is OFF
        if (off_period_ms_ > 0 || blink_color_ != current_color_)
        {
          // Time to blink off
          if (std::chrono::steady_clock::now() > last_on_time_ + std::chrono::milliseconds(on_period_ms_))
          {
            write_led(Turtlebot4LedColor::OFF);
            last_off_time_ = std::chrono::steady_clock::now();
            current_color_ = Turtlebot4LedColor::OFF;
          }
        }
        break;
      }

      default:
      {
        return;
      }
    }

    auto msg = std_msgs::msg::Int32();
    msg.data = static_cast<int32_t>(current_color_);
    led_pub_->publish(msg);
  }
};  

class Leds
{
public:
  Leds(std::shared_ptr<rclcpp::Node> &nh, std::shared_ptr<GpioInterface> gpio_interface, bool use_sim);

  void spin_once();
  void set_led(Turtlebot4LedEnum led, Turtlebot4LedColor color);
  void blink(Turtlebot4LedEnum led, uint32_t blink_period_ms, double duty_cycle, Turtlebot4LedColor color);

private:
  void user_led_callback(const turtlebot4_msgs::msg::UserLed user_led_msg);
  
  std::shared_ptr<rclcpp::Node> nh_;

  rclcpp::Subscription<turtlebot4_msgs::msg::UserLed>::SharedPtr user_led_sub_;

  std::map<Turtlebot4LedEnum, std::shared_ptr<Turtlebot4Led>> leds_;

  std::shared_ptr<GpioInterface> gpio_interface_;

  // Simulation
  bool use_sim_;
};

}  // namespace turtlebot4
