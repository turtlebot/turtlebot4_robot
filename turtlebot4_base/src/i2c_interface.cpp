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

#include "turtlebot4_base/i2c_interface.hpp"

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstring>
#include <string>
#include <iostream>

using turtlebot4_base::I2cInterface;

I2cInterface::I2cInterface(const std::string & i2c_bus)
: i2c_bus_(i2c_bus)
{
}

I2cInterface::I2cInterface(const uint8_t & i2c_bus_number)
: i2c_bus_("/dev/i2c-")
{
  i2c_bus_ += std::to_string(i2c_bus_number);
}

int8_t I2cInterface::open_bus()
{
  i2c_fd_ = open(i2c_bus_.c_str(), O_RDWR);

  if (i2c_fd_ < 0) {
    std::cerr << __PRETTY_FUNCTION__ << ": Could not open: " << i2c_bus_ <<
      " with error: " << strerror(errno) << std::endl;
    close(i2c_fd_);
    return -1;
  } else {
    std::cout << __PRETTY_FUNCTION__ << ": Connected to " << i2c_bus_ << std::endl;
  }
  return 0;
}

int8_t I2cInterface::close_bus()
{
  int ret = close(i2c_fd_);
  if (ret < 0) {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to close : " <<
      strerror(errno) << std::endl;
  }
  return ret;
}

int8_t I2cInterface::set_device_id(const uint8_t device_id)
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);
  int ret = ioctl(i2c_fd_, I2C_SLAVE_FORCE, device_id);
  if (ret < 0) {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to " << static_cast<int>(device_id) <<
      " device: " <<
      strerror(errno) << std::endl;
  }
  return ret;
}

int8_t I2cInterface::read_from_bus(const uint8_t address, void * data, uint32_t count)
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  if (write(i2c_fd_, &address, 1) != 1) {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to write to address: " << strerror(errno) <<
      std::endl;
    return -1;
  }
  int ret = read(i2c_fd_, data, count);

  if (ret < 0) {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to read device(%d): " << strerror(errno) <<
      std::endl;
    return -1;
  } else if (static_cast<uint32_t>(ret) != count) {
    std::cerr << __PRETTY_FUNCTION__ << ": Short read from device, expected" << count <<
      " , got " <<
      ret << std::endl;
    return -1;
  }
  return 0;
}

int8_t I2cInterface::write_to_bus(const uint8_t address)
{
  return write_to_bus(address, nullptr, 0);
}

int8_t I2cInterface::write_to_bus(const uint8_t address, uint8_t byte)
{
  return write_to_bus(address, &byte, 1);
}

int8_t I2cInterface::write_to_bus(void * data, uint32_t count)
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);
  if (count == 0) {
    return 0;
  }

  int ret = write(i2c_fd_, data, count);
  if (ret < 0) {
    return -1;
  } else if (static_cast<uint32_t>(ret) != count) {
    std::cerr << __PRETTY_FUNCTION__ << ": Short write to device, expected " << count << ", got " <<
      ret <<
      std::endl;
    return -1;
  }
  return 0;
}

int8_t I2cInterface::write_to_bus(const uint8_t address, void * data, uint32_t count)
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);
  if (write(i2c_fd_, &address, 1) < 0) {
    return -1;
  }

  if (count == 0) {
    return 0;
  }

  int ret = write(i2c_fd_, data, count);
  if (ret < 0) {
    return -1;
  } else if (static_cast<uint32_t>(ret) != count) {
    std::cerr << __PRETTY_FUNCTION__ << ": Short write to device, expected " << count << ", got " <<
      ret <<
      std::endl;
    return -1;
  }
  return 0;
}

int8_t I2cInterface::write_to_bus(
  const uint8_t device_id, const uint8_t address,
  void * data, uint32_t count)
{
  int8_t ret = 0;
  ret += set_device_id(device_id);
  ret += write_to_bus(address, data, count);
  return ret;
}

int8_t I2cInterface::write_to_bus(const uint8_t device_id, const uint8_t address, uint8_t byte)
{
  return write_to_bus(device_id, address, &byte, 1);
}
