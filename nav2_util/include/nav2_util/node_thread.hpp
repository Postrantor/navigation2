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

#ifndef NAV2_UTIL__NODE_THREAD_HPP_
#define NAV2_UTIL__NODE_THREAD_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace nav2_util {

/*
  这段代码定义了一个 NodeThread
  类，用于在后台线程中处理节点和执行器的回调。它有三个构造函数，分别用于处理节点回调、执行器回调和节点指针回调。
  其中，节点回调和执行器回调的构造函数都需要传入相应的接口指针，而节点指针回调的构造函数则通过
  get_node_base_interface() 函数获取节点接口指针。
  此外，该类还定义了三个成员变量：node_ 为节点接口指针，thread_ 为线程指针，executor_
  为执行器指针。其中，node_ 和 executor_ 可以通过构造函数初始化，而 thread_ 则需要在类内部创建。
*/

/**
 * @class nav2_util::NodeThread
 * @brief 用于处理节点/执行器回调的后台线程
 */
class NodeThread {
public:
  /**
   * @brief 处理节点回调的后台线程构造函数
   * @param node_base 在线程中旋转的节点接口
   */
  explicit NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base);

  /**
   * @brief 处理执行器回调的后台线程构造函数
   * @param executor 在线程中旋转的执行器接口
   */
  explicit NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor);

  /**
   * @brief 处理节点回调的后台线程构造函数
   * @param node 在线程中旋转的节点指针
   */
  template <typename NodeT>
  explicit NodeThread(NodeT node) : NodeThread(node->get_node_base_interface()) {}

  /**
   * @brief 析构函数
   */
  ~NodeThread();

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;  // 节点接口指针
  std::unique_ptr<std::thread> thread_;                         // 线程指针
  rclcpp::Executor::SharedPtr executor_;                        // 执行器指针
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__NODE_THREAD_HPP_
