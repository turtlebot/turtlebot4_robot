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
#include <std_msgs/msg/int32.hpp>
#include "irobot_create_msgs/msg/interface_buttons.hpp"
#include "turtlebot4_msgs/msg/user_button.hpp"

#include "turtlebot4_node/gpio_interface.hpp"
#include "turtlebot4_node/utils.hpp"

namespace turtlebot4
{

  enum Turtlebot4ButtonEnum
  {
    CREATE3_1,
    CREATE3_POWER,
    CREATE3_2,
    HMI_1,
    HMI_2,
    HMI_3,
    HMI_4,
  };

  enum Turtlebot4ButtonState
  {
    RELEASED = 0,
    PRESSED = 1,
    WAIT_FOR_RELEASE
  };

struct Turtlebot4Button
{
  Turtlebot4ButtonEnum button_;

  std::string short_function_;
  turtlebot4_function_callback_t short_cb_;
  std::string long_function_;
  turtlebot4_function_callback_t long_cb_;

  int long_press_duration_ms_;
  std::chrono::time_point<std::chrono::steady_clock> last_start_pressed_time_;

  Turtlebot4ButtonState current_state_;
  Turtlebot4ButtonState next_state_;

  Turtlebot4Button(Turtlebot4ButtonEnum button, std::vector<std::string> params) : button_(button), 
                                                                                   current_state_(RELEASED), 
                                                                                   next_state_(RELEASED)
  {
    short_function_ = params.at(0);
    long_function_ = params.at(1);
    long_press_duration_ms_ = params.at(2).empty() ? 0 : std::stoi(params.at(2));
  }
  
  void set_state(Turtlebot4ButtonState state)
  {
    next_state_ = state;
  }

  void spin_once()
  {
    switch (current_state_)
    {

    case RELEASED:
    {
      if (next_state_ == PRESSED)
      {
        // Start timer
        last_start_pressed_time_ = std::chrono::steady_clock::now();
        current_state_ = PRESSED;
      }
      break;
    }

    case PRESSED:
    {
      if (next_state_ == PRESSED)
      {
        // Long press implemented
        if (long_press_duration_ms_ > 0)
        {
          if (std::chrono::steady_clock::now() > last_start_pressed_time_ + std::chrono::milliseconds(long_press_duration_ms_))
          {
            long_press();
            current_state_ = WAIT_FOR_RELEASE;
          }
        }
        // Long press not implemented, do short press function and wait for release
        else
        {
          short_press();
          current_state_ = WAIT_FOR_RELEASE;
        }
      }
      else if (next_state_ == RELEASED)
      {
        short_press();
        current_state_ = RELEASED;
      }
      break;
    }

    case WAIT_FOR_RELEASE:
    {
      if (next_state_ == RELEASED)
      {
        current_state_ = RELEASED;
      }
      break;
    }

    default:
      break;
    }
  }

  void short_press()
  {
    if (short_cb_ != nullptr)
    {
      short_cb_();
    }
  }

  void long_press()
  {
    if (long_cb_ != nullptr)
    {
      long_cb_();
    }
  }
};  

class Buttons
{
public:
  Buttons(Turtlebot4Model model,
          std::vector<Turtlebot4Button> buttons,
          std::shared_ptr<rclcpp::Node> &nh,
          std::shared_ptr<GpioInterface> gpio_interface, bool use_sim);

  void spin_once();

private:
  void create3_buttons_callback(const irobot_create_msgs::msg::InterfaceButtons::SharedPtr create3_buttons_msg);
  void hmi_buttons_callback(const std_msgs::msg::Int32::SharedPtr hmi_buttons_msg);
  void publish_hmi_buttons();

  uint8_t user_button_pins_[4];

  std::vector<Turtlebot4Button> buttons_;

  std::shared_ptr<GpioInterface> gpio_interface_;

  // Simulation
  bool use_sim_;
  
  std::shared_ptr<rclcpp::Node> nh_;
  rclcpp::Subscription<irobot_create_msgs::msg::InterfaceButtons>::SharedPtr create3_buttons_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hmi_buttons_sub_;
  rclcpp::Publisher<turtlebot4_msgs::msg::UserButton>::SharedPtr hmi_buttons_pub_;
  Turtlebot4Model model_;
};

}  // namespace turtlebot4
