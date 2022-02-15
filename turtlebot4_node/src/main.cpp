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

#include <rcutils/cmdline_parser.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>

#include "turtlebot4_node/turtlebot4.hpp"
#include "turtlebot4_node/utils.hpp"

void help_print()
{
  printf("For turtlebot4 node : \n");
  printf("turtlebot4_node [-h] [--use_sim {bool}]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("--use_sim : Using simulated Turtlebot4.\n");
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    help_print();
    return 0;
  }

  std::string use_sim_arg;
  if (rcutils_cli_option_exist(argv, argv + argc, "--use_sim")) {
    use_sim_arg = std::string(rcutils_cli_get_option(argv, argv + argc, "--use_sim"));
  }

  std::string model_arg;
  if (rcutils_cli_option_exist(argv, argv + argc, "--model")) {
    model_arg = std::string(rcutils_cli_get_option(argv, argv + argc, "--model"));
  }

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  bool use_sim = use_sim_arg == "true" ? true : false;

  turtlebot4::Turtlebot4Model model = turtlebot4::Turtlebot4Model::STANDARD;

  if (model_arg == "lite") {
    model = turtlebot4::Turtlebot4Model::LITE;
  }

  auto turtlebot4 = std::make_shared<turtlebot4::Turtlebot4>(use_sim, model);

  executor.add_node(turtlebot4);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
