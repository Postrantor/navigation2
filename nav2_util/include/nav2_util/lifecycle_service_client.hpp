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

#ifndef NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_
#define NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/service_client.hpp"

namespace nav2_util {

// clang-format off
/*
  该代码段定义了一个名为`LifecycleServiceClient`的类，用于与生命周期节点交互。其中包含了以下公共成员函数：

  - `explicit LifecycleServiceClient(const std::string& lifecycle_node_name)`：构造函数，创建一个`LifecycleServiceClient`对象。
  - `LifecycleServiceClient(const std::string& lifecycle_node_name, rclcpp::Node::SharedPtr parent_node)`：构造函数，创建一个`LifecycleServiceClient`对象。
  - `bool change_state(const uint8_t transition, const std::chrono::seconds timeout)`：触发状态转换。
  - `bool change_state(std::uint8_t transition)`：触发状态转换并返回结果。
  - `uint8_t get_state(const std::chrono::seconds timeout = std::chrono::seconds(2))`：获取当前状态。

  其中，`change_state`和`get_state`函数都需要与生命周期节点进行通信，因此使用了ROS2中提供的服务客户端（`ServiceClient`）来实现。
  在`LifecycleServiceClient`类中，还包含了一个指向生命周期节点的智能指针`node_`，以及两个服务客户端`change_state_`和`get_state_`。
*/
// clang-format on

/// 用于与生命周期节点交互的辅助函数。
class LifecycleServiceClient {
public:
  /**
   * @brief 构造函数，创建一个LifecycleServiceClient对象。
   * @param lifecycle_node_name 生命周期节点名称。
   */
  explicit LifecycleServiceClient(const std::string& lifecycle_node_name);

  /**
   * @brief 构造函数，创建一个LifecycleServiceClient对象。
   * @param lifecycle_node_name 生命周期节点名称。
   * @param parent_node 父节点。
   */
  LifecycleServiceClient(
      const std::string& lifecycle_node_name,  //
      rclcpp::Node::SharedPtr parent_node);

  /// 这里带了一个 timeout 挺好
  /**
   * @brief 触发状态转换。
   * @param transition 转换ID，类型为lifecycle_msgs::msg::Transition。
   * @param timeout 超时时间。
   * @return 如果成功，则返回true，否则抛出std::runtime_error异常。
   */
  bool change_state(
      const uint8_t transition,  // takes a lifecycle_msgs::msg::Transition id
      const std::chrono::seconds timeout);

  /**
   * @brief 触发状态转换并返回结果。
   * @param transition 转换ID，类型为lifecycle_msgs::msg::Transition。
   * @return 如果成功，则返回true，否则返回false。
   */
  bool change_state(std::uint8_t transition);

  /**
   * @brief 获取当前状态。
   * @param timeout 超时时间，默认为2秒。
   * @return 返回当前状态，类型为lifecycle_msgs::msg::State。
   * @throws 如果失败，则抛出std::runtime_error异常。
   */
  uint8_t get_state(const std::chrono::seconds timeout = std::chrono::seconds(2));

protected:
  rclcpp::Node::SharedPtr node_;                                  // 生命周期节点
  ServiceClient<lifecycle_msgs::srv::ChangeState> change_state_;  // 触发状态转换的服务客户端
  ServiceClient<lifecycle_msgs::srv::GetState> get_state_;  // 获取当前状态的服务客户端
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_SERVICE_CLIENT_HPP_
