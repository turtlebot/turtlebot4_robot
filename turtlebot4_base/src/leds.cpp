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

#include "turtlebot4_base/leds.hpp"

#include <memory>

using turtlebot4_base::Leds;
using turtlebot4::Turtlebot4LedEnum;

Leds::Leds(
  std::shared_ptr<rclcpp::Node> & nh,
  std::shared_ptr<GpioInterface> gpio_interface)
: nh_(nh),
  gpio_interface_(gpio_interface)
{
  RCLCPP_INFO(nh_->get_logger(), "Leds Init");

  nh_->declare_parameter("gpio.led_green_power", 17);
  nh_->declare_parameter("gpio.led_green_motors", 18);
  nh_->declare_parameter("gpio.led_green_comms", 27);
  nh_->declare_parameter("gpio.led_green_wifi", 24);
  nh_->declare_parameter("gpio.led_green_battery", 22);
  nh_->declare_parameter("gpio.led_red_battery", 23);
  nh_->declare_parameter("gpio.led_green_user_1", 25);
  nh_->declare_parameter("gpio.led_green_user_2", 6);
  nh_->declare_parameter("gpio.led_red_user_2", 12);

  // Power
  leds_ = {
    {Turtlebot4LedEnum::POWER, std::make_shared<Turtlebot4Led>(
        gpio_interface_,
        nh_->get_parameter("gpio.led_green_power").as_int())},
    {Turtlebot4LedEnum::MOTORS, std::make_shared<Turtlebot4Led>(
        gpio_interface_,
        nh_->get_parameter("gpio.led_green_motors").as_int())},
    {Turtlebot4LedEnum::COMMS, std::make_shared<Turtlebot4Led>(
        gpio_interface_,
        nh_->get_parameter("gpio.led_green_comms").as_int())},
    {Turtlebot4LedEnum::WIFI, std::make_shared<Turtlebot4Led>(
        gpio_interface_,
        nh_->get_parameter("gpio.led_green_wifi").as_int())},
    {Turtlebot4LedEnum::BATTERY, std::make_shared<Turtlebot4Led>(
        gpio_interface_,
        nh_->get_parameter("gpio.led_green_battery").as_int(),
        nh_->get_parameter("gpio.led_red_battery").as_int())},
    {Turtlebot4LedEnum::USER_1, std::make_shared<Turtlebot4Led>(
        gpio_interface_,
        nh_->get_parameter("gpio.led_green_user_1").as_int())},
    {Turtlebot4LedEnum::USER_2, std::make_shared<Turtlebot4Led>(
        gpio_interface_,
        nh_->get_parameter("gpio.led_green_user_2").as_int(),
        nh_->get_parameter("gpio.led_red_user_2").as_int())},
  };

  leds_[Turtlebot4LedEnum::POWER]->create_subscription(nh_, "hmi/led/_power");
  leds_[Turtlebot4LedEnum::MOTORS]->create_subscription(nh_, "hmi/led/_motors");
  leds_[Turtlebot4LedEnum::COMMS]->create_subscription(nh_, "hmi/led/_comms");
  leds_[Turtlebot4LedEnum::WIFI]->create_subscription(nh_, "hmi/led/_wifi");
  leds_[Turtlebot4LedEnum::BATTERY]->create_subscription(nh_, "hmi/led/_battery");
  leds_[Turtlebot4LedEnum::USER_1]->create_subscription(nh_, "hmi/led/_user1");
  leds_[Turtlebot4LedEnum::USER_2]->create_subscription(nh_, "hmi/led/_user2");
}
