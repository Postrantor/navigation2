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

#include <memory>

#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief 主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数列表
 * @details 该函数初始化 ROS 2 节点，创建 nav2_lifecycle_manager::LifecycleManager 实例，
 *         并通过 rclcpp::spin() 进入 ROS 2 等待循环，最后关闭节点并返回 0。
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_lifecycle_manager::LifecycleManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
