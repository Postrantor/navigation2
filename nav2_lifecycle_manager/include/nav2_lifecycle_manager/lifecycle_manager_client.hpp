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

#ifndef NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_
#define NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav2_util/service_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

/**
  参数列表：

  - `name`：管理节点名称。
  - `parent_node`：执行服务调用的节点。
  - `timeout`：服务调用超时时间。

  代码功能总结：

  这段代码定义了一个名为 `LifecycleManagerClient` 的类，该类提供了一系列方法，用于向
  LifecycleManager 发送请求以控制导航模块的生命周期状态。具体来说，它提供了以下方法：

  - `startup()`：调用启动服务。
  - `shutdown()`：调用关闭服务。
  - `pause()`：调用暂停服务。
  - `resume()`：调用恢复服务。
  - `reset()`：调用重置服务。
  - `is_active()`：检查生命周期节点管理器服务器是否处于活动状态。
  - `set_initial_pose()`：设置带协方差的初始姿态。
  - `navigate_to_pose()`：将目标姿态发送到 NavigationToPose 动作服务器。

  此外，还有一些私有方法和成员变量，用于服务调用和节点管理。
 */

namespace nav2_lifecycle_manager {

/**
 * @enum nav2_lifecycle_manager::SystemStatus
 * @brief 表示系统状态的枚举类。
 */
enum class SystemStatus { ACTIVE, INACTIVE, TIMEOUT };

/**
 * @class nav2_lifecycle_manager::LifeCycleMangerClient
 * @brief LifecycleManagerClient 发送请求到 LifecycleManager 控制导航模块的生命周期状态。
 */
class LifecycleManagerClient {
public:
  /**
   * @brief LifeCycleMangerClient 的构造函数。
   * @param name 管理节点名称。
   * @param parent_node 执行服务调用的节点。
   */
  explicit LifecycleManagerClient(
      const std::string& name, std::shared_ptr<rclcpp::Node> parent_node);

  // Nav2 生命周期管理器的客户端接口
  /**
   * @brief 调用启动服务。
   * @return 返回 true 或 false。
   */
  bool startup(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  /**
   * @brief 调用关闭服务。
   * @return 返回 true 或 false。
   */
  bool shutdown(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  /**
   * @brief 调用暂停服务。
   * @return 返回 true 或 false。
   */
  bool pause(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  /**
   * @brief 调用恢复服务。
   * @return 返回 true 或 false。
   */
  bool resume(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  /**
   * @brief 调用重置服务。
   * @return 返回 true 或 false。
   */
  bool reset(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  /**
   * @brief 检查生命周期节点管理器服务器是否处于活动状态。
   * @return 返回 ACTIVE 或 INACTIVE 或 TIMEOUT。
   */
  SystemStatus is_active(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  // 为便于脚本测试而提供的一些方便方法
  /**
   * @brief 设置带协方差的初始姿态。
   * @param x X 坐标。
   * @param y Y 坐标。
   * @param theta 方向。
   */
  void set_initial_pose(double x, double y, double theta);
  /**
   * @brief 将目标姿态发送到 NavigationToPose 动作服务器。
   * @param x X 坐标。
   * @param y Y 坐标。
   * @param theta 方向。
   * @return 返回 true 或 false。
   */
  bool navigate_to_pose(double x, double y, double theta);

protected:
  using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;

  /**
   * @brief 用于调用启动、关闭等通用方法。
   * @param command 命令。
   */
  bool callService(
      uint8_t command,  //
      const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  // 用于服务调用的节点
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<nav2_util::ServiceClient<ManageLifecycleNodes>> manager_client_;
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::Trigger>> is_active_client_;
  std::string manage_service_name_;
  std::string active_service_name_;
};

}  // namespace nav2_lifecycle_manager

#endif  // NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_
