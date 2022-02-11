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
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>

#include "turtlebot4_node/ssd1306.hpp"
#include "turtlebot4_node/utils.hpp"
#include "turtlebot4_node/buttons.hpp"

#include <memory>

namespace turtlebot4
{

struct Turtlebot4MenuEntry
{
    std::string name_;
    turtlebot4_function_callback_t cb_;

    Turtlebot4MenuEntry(std::string name) : name_(name)
    {}
};

class Display
{
public:
    // Constructor and Destructor
    explicit Display(std::vector<Turtlebot4MenuEntry> entries,
                     std::shared_ptr<rclcpp::Node> &nh, std::shared_ptr<I2cInterface> i2c, 
                     std::shared_ptr<GpioInterface> gpio, bool use_sim);
    virtual ~Display() {}

    // Setters
    void set_battery(const sensor_msgs::msg::BatteryState::SharedPtr & battery_state_msg);
    void set_ip(std::string ip);
    void add_line(std::string data);
    void pad_line(std::string & line);
    void scroll_up();
    void scroll_down();
    void select();
    void back();
    void show_message(std::vector<std::string> message);
    void show_message(std::string message);

    // Spin Once
    void spin_once();

private:
    // Update display
    void update();

    void update_header();
    void draw_header();

    void set_menu_entries();

    std::vector<Turtlebot4MenuEntry> get_visible_entries();

    void display_callback(const std_msgs::msg::String::SharedPtr display_msg);


    // Node handle
    std::shared_ptr<rclcpp::Node> nh_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr display_raw_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr display_selected_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr display_set_sub_;

    // Menu
    std::vector<Turtlebot4MenuEntry> menu_entries_;
    std::vector<Turtlebot4MenuEntry> visible_entries_;
    std::vector<std::string> display_lines_;
    bool menu_override_;
    uint8_t scroll_position_;
    uint8_t selected_line_;

    // Header
    std::string ip_;
    int battery_percentage_;
    std::string header_;

    // Display driver
    Ssd1306 oled_;

    // Use simulation
    bool use_sim_;
};

} // namespace turtlebot4
