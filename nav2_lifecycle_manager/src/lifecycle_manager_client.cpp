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

#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "nav2_util/geometry_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_lifecycle_manager {

/*
  其中 LifecycleManagerClient
  类提供了启动、关闭、暂停、恢复和重置生命周期管理器的功能，这些功能都是通过调用 callService
  函数实现的。在构造函数中，会根据节点名称和父节点指针创建服务客户端。每个服务都有一个超时时间，如果在规定时间内没有得到响应，则会返回错误。
*/
using nav2_util::geometry_utils::orientationAroundZAxis;

/**
 * @brief Lifecycle Manager Client 类的构造函数
 * @param name 节点名称
 * @param parent_node 父节点指针
 */
LifecycleManagerClient::LifecycleManagerClient(
    const std::string &name,  //
    std::shared_ptr<rclcpp::Node> parent_node) {
  manage_service_name_ = name + std::string("/manage_nodes");
  active_service_name_ = name + std::string("/is_active");

  // Use parent node for service call and logging
  node_ = parent_node;

  // Create the service clients
  manager_client_ =
      std::make_shared<nav2_util::ServiceClient<ManageLifecycleNodes>>(manage_service_name_, node_);
  is_active_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::Trigger>>(
      active_service_name_, node_);
}

/** ========= ========= ========= //
// ========= ========= ========= **/

/**
 * @brief 启动生命周期管理器
 * @param timeout 超时时间
 * @return bool 是否启动成功
 */
bool LifecycleManagerClient::startup(const std::chrono::nanoseconds timeout) {
  return callService(ManageLifecycleNodes::Request::STARTUP, timeout);
}

/**
 * @brief 关闭生命周期管理器
 * @param timeout 超时时间
 * @return bool 是否关闭成功
 */
bool LifecycleManagerClient::shutdown(const std::chrono::nanoseconds timeout) {
  return callService(ManageLifecycleNodes::Request::SHUTDOWN, timeout);
}

/**
 * @brief 暂停生命周期管理器
 * @param timeout 超时时间
 * @return bool 是否暂停成功
 */
bool LifecycleManagerClient::pause(const std::chrono::nanoseconds timeout) {
  return callService(ManageLifecycleNodes::Request::PAUSE, timeout);
}

/**
 * @brief 恢复生命周期管理器
 * @param timeout 超时时间
 * @return bool 是否恢复成功
 */
bool LifecycleManagerClient::resume(const std::chrono::nanoseconds timeout) {
  return callService(ManageLifecycleNodes::Request::RESUME, timeout);
}

/**
 * @brief 重置生命周期管理器
 * @param timeout 超时时间
 * @return bool 是否重置成功
 */
bool LifecycleManagerClient::reset(const std::chrono::nanoseconds timeout) {
  return callService(ManageLifecycleNodes::Request::RESET, timeout);
}

/** ========= ========= ========= //
// ========= ========= ========= **/

/*
  上述代码是在 ROS2 Navigation2 组件中的 LifecycleManagerClient 相关代码。其中包含两个函数，分别为
  is_active 和 callService。

  is_active 函数用于判断 LifecycleManager 是否处于 ACTIVE 状态。该函数首先构造 Trigger 请求，等待
  is_active 服务可用，发送请求并获取响应，根据响应的 success 字段判断是否处于 ACTIVE 状态。

  callService 函数用于调用 LifecycleManager 的 ManageLifecycleNodes 服务。该函数首先构造
  ManageLifecycleNodes 请求，等待 manager 服务可用，发送请求并获取 future_result，返回 future_result
  的 success 字段。
*/
/**
 * @brief 判断 LifecycleManager 是否处于 ACTIVE 状态
 * @param timeout 超时时间
 * @return SystemStatus 系统状态
 * @details 构造 Trigger 请求，等待 is_active 服务可用，发送请求并获取响应，根据响应的 success
 * 字段判断是否处于 ACTIVE 状态。
 */
SystemStatus LifecycleManagerClient::is_active(const std::chrono::nanoseconds timeout) {
  // 构造 Trigger 请求
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();

  // 打印日志信息
  RCLCPP_DEBUG(node_->get_logger(), "Waiting for the %s service...", active_service_name_.c_str());

  // 从这里其实可以看出，这个manager就是demos中的那个脚本的角色
  // 等待 is_active 服务可用
  if (!is_active_client_->wait_for_service(std::chrono::seconds(1))) {
    return SystemStatus::TIMEOUT;
  }

  // 打印日志信息
  RCLCPP_DEBUG(node_->get_logger(), "Sending %s request", active_service_name_.c_str());

  try {
    // 发送请求并获取响应
    response = is_active_client_->invoke(request, timeout);
  } catch (std::runtime_error &) {
    return SystemStatus::TIMEOUT;
  }

  // 根据响应的 success 字段判断是否处于 ACTIVE 状态
  if (response->success) {
    return SystemStatus::ACTIVE;
  } else {
    return SystemStatus::INACTIVE;
  }
}

/**
 * @brief 调用 LifecycleManager 的 ManageLifecycleNodes 服务
 * @param command 命令类型
 * @param timeout 超时时间
 * @return bool 是否成功调用服务
 * @details 构造 ManageLifecycleNodes 请求，等待 manager 服务可用，发送请求并获取
 * future_result，返回 future_result 的 success 字段。
 */
bool LifecycleManagerClient::callService(uint8_t command, const std::chrono::nanoseconds timeout) {
  // 构造 ManageLifecycleNodes 请求
  auto request = std::make_shared<ManageLifecycleNodes::Request>();
  request->command = command;

  // 打印日志信息
  RCLCPP_DEBUG(node_->get_logger(), "Waiting for the %s service...", manage_service_name_.c_str());

  // 等待 manager 服务可用
  while (!manager_client_->wait_for_service(timeout)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "Client interrupted while waiting for service to appear");
      return false;
    }
    RCLCPP_DEBUG(node_->get_logger(), "Waiting for service to appear...");
  }

  RCLCPP_DEBUG(node_->get_logger(), "Sending %s request", manage_service_name_.c_str());
  // 发送请求并获取 future_result
  try {
    auto future_result = manager_client_->invoke(request, timeout);
    return future_result->success;
  } catch (std::runtime_error &) {
    return false;
  }
}

}  // namespace nav2_lifecycle_manager
