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

#ifndef TURTLEBOT4_BASE__GPIO_INTERFACE_HPP_
#define TURTLEBOT4_BASE__GPIO_INTERFACE_HPP_

#include <linux/gpio.h>
#include <gpiod.h>

#include <string>
#include <mutex>
#include <memory>
#include <map>


namespace turtlebot4_base
{

enum GpioInterfaceLineDirection
{
  LINE_DIRECTION_INPUT = GPIOD_LINE_DIRECTION_INPUT,
  LINE_DIRECTION_OUTPUT = GPIOD_LINE_DIRECTION_OUTPUT
};

class GpioInterface
{
public:
  explicit GpioInterface(const std::string & gpio_chip);
  explicit GpioInterface(const uint8_t & gpio_chip_number);

  void write(uint8_t line, uint8_t value);
  uint8_t read(uint8_t line);

  void open_chip();
  void close_chip();

  void add_line(uint8_t line, GpioInterfaceLineDirection direction);

private:
  std::string gpio_chip_;

  gpiod_chip * chip_;

  std::map<uint8_t, gpiod_line *> lines_;
};

}  // namespace turtlebot4_base

#endif  // TURTLEBOT4_BASE__GPIO_INTERFACE_HPP_
