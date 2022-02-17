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

#include "turtlebot4_base/gpio_interface.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <iostream>
#include <string>
#include <utility>

using turtlebot4_base::GpioInterface;

GpioInterface::GpioInterface(const std::string & gpio_chip)
: gpio_chip_(gpio_chip)
{
  open_chip();
}

GpioInterface::GpioInterface(const uint8_t & gpio_chip_number)
: gpio_chip_("gpiochip")
{
  gpio_chip_ += std::to_string(gpio_chip_number);
  open_chip();
}

/**
 * @brief Open the gpio chip
 */
void GpioInterface::open_chip()
{
  chip_ = gpiod_chip_open_by_name(gpio_chip_.c_str());
}

/**
 * @brief Close the gpio chip
 */
void GpioInterface::close_chip()
{
  for (auto line : lines_) {
    gpiod_line_release(line.second);
  }

  gpiod_chip_close(chip_);
}

/**
 * @brief Add a new gpio line
 */
void GpioInterface::add_line(uint8_t line, GpioInterfaceLineDirection direction)
{
  gpiod_line * gpio_line = gpiod_chip_get_line(chip_, line);

  if (gpio_line != nullptr) {
    lines_.insert(std::pair<uint8_t, gpiod_line *>(line, gpio_line));
  } else {
    std::cerr << "Invalid GPIO Line" << std::endl;
    return;
  }

  if (direction == LINE_DIRECTION_INPUT) {
    gpiod_line_request_input(lines_[line], "Turtlebot4");
  } else if (direction == LINE_DIRECTION_OUTPUT) {
    gpiod_line_request_output(lines_[line], "Turtlebot4", 0);
  } else {
    std::cerr << "Invalid GPIO Line Direction" << std::endl;
  }
}

/**
 * @brief Write value to gpio line
 */
void GpioInterface::write(uint8_t line, uint8_t value)
{
  gpiod_line_set_value(lines_[line], value);
}

/**
 * @brief Read value from gpio line
 */
uint8_t GpioInterface::read(uint8_t line)
{
  return gpiod_line_get_value(lines_[line]);
}
