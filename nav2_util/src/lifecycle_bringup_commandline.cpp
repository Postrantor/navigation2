// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "nav2_util/lifecycle_utils.hpp"
#include "rclcpp/rclcpp.hpp"

using std::cerr;
using namespace std::chrono_literals;

/**
 * @brief 输出命令行参数错误信息并退出程序
 * @details 如果命令行参数不正确，则输出错误信息并退出程序
 */
void usage() {
  cerr << "Invalid command line.\n\n";
  cerr << "This command will take a set of unconfigured lifecycle nodes through the\n";
  cerr << "CONFIGURED to the ACTIVATED state\n";
  cerr << "The nodes are brought up in the order listed on the command line\n\n";
  cerr << "Usage:\n";
  cerr << " > lifecycle_startup <node name> ...\n";
  std::exit(1);
}

/**
 * @brief 生命周期管理器启动程序的主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数列表
 * @return 程序退出状态码
 * @details 解析命令行参数，初始化 ROS2 节点，启动生命周期节点，并在完成后关闭 ROS2 节点
 */
int main(int argc, char* argv[]) {
  // 检查命令行参数是否正确
  if (argc == 1) {
    usage();
  }
  // 初始化 ROS2 节点
  rclcpp::init(0, nullptr);
  // 启动生命周期节点
  nav2_util::startup_lifecycle_nodes(std::vector<std::string>(argv + 1, argv + argc), 10s);
  // 关闭 ROS2 节点
  rclcpp::shutdown();
}
