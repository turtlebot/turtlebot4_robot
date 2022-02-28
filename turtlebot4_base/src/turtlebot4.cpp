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

#include "turtlebot4_base/turtlebot4.hpp"

#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <utility>

using turtlebot4_base::Turtlebot4;

/**
 * @brief Turtlebot4 Node constructor
 */
Turtlebot4::Turtlebot4()
: Node("turtlebot4_base_node",
    rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(get_logger(), "Turtlebot4 Base Node Running");

  // Create node handle
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  gpiochip0_ = std::make_shared<GpioInterface>(0);

  i2c3_ = std::make_shared<I2cInterface>(3);

  // Buttons
  buttons_ = std::make_unique<Buttons>(node_handle_, gpiochip0_);

  // Display
  display_ = std::make_unique<Display>(node_handle_, i2c3_, gpiochip0_);

  // Leds
  leds_ = std::make_unique<Leds>(node_handle_, gpiochip0_);

  run();
}

/**
 * @brief Turtlebot4 Node run
 */
void Turtlebot4::run()
{
  buttons_timer(std::chrono::milliseconds(10));
}

/**
 * @brief Creates and runs timer to poll buttons
 * @input timeout - Sets timer period in milliseconds
 */
void Turtlebot4::buttons_timer(const std::chrono::milliseconds timeout)
{
  buttons_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      buttons_->spin_once();
    });
}
