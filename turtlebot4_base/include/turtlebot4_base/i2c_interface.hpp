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

#ifndef TURTLEBOT4_BASE__I2C_INTERFACE_HPP_
#define TURTLEBOT4_BASE__I2C_INTERFACE_HPP_

#include <string>
#include <mutex>

namespace turtlebot4_base
{

class I2cInterface
{
public:
  explicit I2cInterface(const std::string & i2c_bus);
  explicit I2cInterface(const uint8_t & i2c_bus_number);

  int8_t open_bus();
  int8_t close_bus();
  int8_t set_device_id(uint8_t device_id);
  int8_t read_from_bus(uint8_t address, void * data, uint32_t count);
  int8_t read_from_bus(
    uint8_t device_id, uint8_t address,
    void * data, uint32_t count);
  int8_t write_to_bus(const uint8_t address);
  int8_t write_to_bus(const uint8_t address, uint8_t byte);
  int8_t write_to_bus(const uint8_t address, void * data, uint32_t count);
  int8_t write_to_bus(
    const uint8_t device_id, const uint8_t address,
    void * data, uint32_t count);
  int8_t write_to_bus(const uint8_t device_id, const uint8_t address, uint8_t byte);
  int8_t write_to_bus(void * data, uint32_t count);

private:
  std::string i2c_bus_;
  int i2c_fd_;
  std::mutex i2c_mutex_;
};

}  // namespace turtlebot4_base

#endif  // TURTLEBOT4_BASE__I2C_INTERFACE_HPP_
