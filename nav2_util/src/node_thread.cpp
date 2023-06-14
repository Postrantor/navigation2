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

#include "nav2_util/node_thread.hpp"

#include <memory>

namespace nav2_util {

// clang-format off
/*
  该代码定义了一个名为NodeThread的类，该类用于在单独的线程中执行ROS2节点。
  该类有两个构造函数，一个接受节点基础接口的指针，另一个接受单线程执行器的指针。
  - 当传入节点基础接口的指针时，NodeThread对象将创建一个单线程执行器，并将节点添加到执行器中并开始执行spin()函数，直到取消执行器并等待线程结束。
  - 当传入单线程执行器的指针时，NodeThread对象将执行器添加到当前对象的executor_成员变量中，并开始执行spin()函数，直到取消执行器并等待线程结束。
  - 析构函数用于取消执行器并等待线程结束。
*/
// clang-format on

/**
 * @brief NodeThread类的构造函数，传入节点基础接口的指针
 * @param node_base 节点基础接口的指针
 * @details
 *     创建一个单线程执行器，将节点添加到执行器中并开始执行spin()函数，直到取消执行器并等待线程结束
 */
NodeThread::NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
    : node_(node_base) {
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  thread_ = std::make_unique<std::thread>([&]() {
    executor_->add_node(node_);
    executor_->spin();
    executor_->remove_node(node_);
  });
}

/**
 * @brief NodeThread类的构造函数，传入单线程执行器的指针
 * @param executor 单线程执行器的指针
 * @details
 * 将执行器添加到当前对象的executor_成员变量中，并开始执行spin()函数，直到取消执行器并等待线程结束
 */
NodeThread::NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor)
    : executor_(executor) {
  thread_ = std::make_unique<std::thread>([&]() { executor_->spin(); });
}

/**
 * @brief NodeThread类的析构函数
 * @details 取消执行器并等待线程结束
 */
NodeThread::~NodeThread() {
  executor_->cancel();
  thread_->join();
}

}  // namespace nav2_util
