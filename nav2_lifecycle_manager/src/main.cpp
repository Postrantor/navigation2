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
 * @brief nav2_planner 组件的主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数列表
 * @details 该函数初始化 ROS 2 节点，创建 nav2_lifecycle_manager::LifecycleManager 实例，
 *          并通过 rclcpp::spin() 进入 ROS 2 等待循环，最后关闭节点并返回 0。
 * 1. 调用 rclcpp::init() 进行 ROS 2 初始化
 * 2. 创建一个 nav2_lifecycle_manager::LifecycleManager 类型的节点实例 node
 * 3. 调用 rclcpp::spin() 进入 ROS 2 等待循环，处理节点间通信
 * 4. 当节点被终止时，调用 rclcpp::shutdown() 关闭 ROS 2
 * 5. 返回值为 0 表示程序正常结束
 * 该代码段是nav2_planner组件的主函数，主要功能是初始化ROS2节点，创建nav2_lifecycle_manager::LifecycleManager对象，然后进入spin循环等待节点结束。具体来说，该函数首先调用rclcpp::init函数初始化ROS2节点，接着使用std::make_shared函数创建一个nav2_lifecycle_manager::LifecycleManager对象，并将其赋值给node智能指针。最后，调用rclcpp::spin函数进入spin循环等待节点结束，当节点结束时，调用rclcpp::shutdown函数关闭ROS2节点。
 */
int main(int argc, char** argv) {
  // 初始化 ROS 2 节点
  rclcpp::init(argc, argv);
  // 创建 nav2_lifecycle_manager::LifecycleManager 实例
  auto node = std::make_shared<nav2_lifecycle_manager::LifecycleManager>();
  // 进入 ROS 2 等待循环
  rclcpp::spin(node);
  // 关闭节点
  rclcpp::shutdown();

  return 0;
}
