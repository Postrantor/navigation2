// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2022 Samsung Research America
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

#ifndef NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "bondcpp/bond.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav2_util/lifecycle_service_client.hpp"
#include "nav2_util/node_thread.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace nav2_lifecycle_manager {
using namespace std::chrono_literals;  // NOLINT

using nav2_msgs::srv::ManageLifecycleNodes;

/*
  该类实现了一个服务接口，用于转换Nav2堆栈的生命周期节点。它接收转换请求，然后使用生命周期接口来更改生命周期节点的状态。

  该类继承自rclcpp::Node，具有一个构造函数和一个析构函数。其中构造函数带有一个可选参数options，用于控制节点的创建。该类还包括一个回调组callback_group_和一个服务线程service_thread_，以及两个服务manager_srv_和is_active_srv_。其中manager_srv_是用于管理生命周期节点的服务，is_active_srv_则用于检查节点是否处于活动状态。

  此外，该类还包括一个managerCallback()函数，用于处理生命周期节点管理器的回调函数。该函数接收服务请求，并使用生命周期接口更改生命周期节点的状态。
*/
/**
 * @class nav2_lifecycle_manager::LifecycleManager
 * @brief
 * 实现了Nav2堆栈的生命周期节点的服务接口。它接收转换请求，然后使用生命周期接口来更改生命周期节点的状态。
 */
class LifecycleManager : public rclcpp::Node {
public:
  /**
   * @brief nav2_lifecycle_manager::LifecycleManager的构造函数
   * @param options 控制节点创建的附加选项。
   */
  explicit LifecycleManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  /**
   * @brief nav2_lifecycle_manager::LifecycleManager的析构函数
   */
  ~LifecycleManager();

protected:
  // 用于服务和定时器的回调组
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::unique_ptr<nav2_util::NodeThread> service_thread_;

  // 此节点提供的服务
  rclcpp::Service<ManageLifecycleNodes>::SharedPtr manager_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr is_active_srv_;
  /**
   * @brief 生命周期节点管理器回调函数
   * @param request_header 服务请求的头部
   * @param request 服务请求
   * @param reponse 服务响应
   */
  void managerCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<ManageLifecycleNodes::Request> request,
      std::shared_ptr<ManageLifecycleNodes::Response> response);

  /**
   * @brief isActiveCallback函数检查被管理的节点是否处于活动状态。
   * @param request_header 请求头
   * @param request 服务请求
   * @param response 服务响应
   * @details
   * 此函数是一个回调函数，用于检查被管理的节点是否处于活动状态。如果所有节点都处于活动状态，则返回true；否则返回false。
   */
  void isActiveCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // 支持服务调用的函数
  /**
   * @brief 启动被管理的节点。
   * @return true或false
   */
  bool startup();

  /**
   * @brief 停用、清理和关闭所有被管理的节点。
   * @return true或false
   */
  bool shutdown();

  /**
   * @brief 重置所有被管理的节点。
   * @return true或false
   */
  bool reset(bool hard_reset = false);

  /**
   * @brief 暂停所有被管理的节点。
   * @return true或false
   */
  bool pause();

  /**
   * @brief 恢复所有被管理的节点。
   * @return true或false
   */
  bool resume();

  /**
   * @brief 在我们的Context关闭之前执行preshutdown活动。
   * 注意，这与我们的Context的关闭序列有关，而不是生命周期节点状态机或shutdown()。
   */
  void onRclPreshutdown();

  // 创建服务客户端的支持函数
  void createLifecycleServiceClients();

  // 支持关闭的函数
  void shutdownAllNodes();

  // 销毁所有生命周期服务客户端。
  void destroyLifecycleServiceClients();

  // 创建绑定计时器的支持函数
  void createBondTimer();

  // 创建绑定连接的支持函数
  /**
   * @brief 创建绑定连接的支持函数
   * @param node_name 节点名称
   * @return 如果创建成功，则返回 true，否则返回 false
   */
  bool createBondConnection(const std::string& node_name);

  // 杀死绑定连接的支持函数
  /**
   * @brief 杀死绑定连接的支持函数
   */
  void destroyBondTimer();

  // 检查绑定连接的支持函数
  /**
   * @brief 检查绑定连接的支持函数
   * 如果有非响应性的情况，将会关闭系统
   */
  void checkBondConnections();

  // 检查绑定连接是否在重启后恢复的支持函数
  /**
   * @brief 检查绑定连接是否在重启后恢复的支持函数
   * 如果从非响应性转换为响应性，将会重新启动系统
   */
  void checkBondRespawnConnection();

  /**
   * @brief 对于一个节点，将其状态转换到新的目标状态
   * @param node_name 节点名称
   * @param transition 目标状态
   * @return 如果转换成功，则返回 true，否则返回 false
   */
  bool changeStateForNode(const std::string& node_name, std::uint8_t transition);

  /**
   * @brief 对于映射中的每个节点，将其状态转换到新的目标状态
   * @param transition 目标状态
   * @param hard_change 是否强制更改
   * @return 如果转换成功，则返回 true，否则返回 false
   */
  bool changeStateForAllNodes(std::uint8_t transition, bool hard_change = false);

  // 在控制台上突出显示输出的辅助函数
  /**
   * @brief 在控制台上突出显示输出的辅助函数
   * @param msg 输出信息
   */
  void message(const std::string& msg);

  // 诊断函数
  /**
   * @brief 检查 Nav2 系统是否处于活动状态的函数
   * @param stat 诊断状态包装器
   */
  void CreateActiveDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

  /**
   * 注册此节点的 rcl 上下文的 preshutdown 回调。
   * 回调在此节点的上下文关闭之前触发。
   * 注意，这与生命周期状态机或 shutdown() 实例函数没有直接关系。
   */
  void registerRclPreshutdownCallback();

  /*
    - init_timer_：初始化定时器，用于检查节点的连接情况。
    - bond_timer_：绑定定时器，用于检查绑定状态。
    - bond_respawn_timer_：绑定重生定时器，用于在绑定超时后重新连接。
    - bond_timeout_：绑定超时时间。
    - bond_map_：所有节点的绑定映射表。
    - node_map_：所有节点的生命周期服务客户端映射表。
    - transition_label_map_：状态转换标签映射表。
    - transition_state_map_：预期的状态转换到主状态的映射表。
    - node_names_：要管理的节点名称，按照期望的启动顺序排列。
    - autostart_：是否自动启动系统。
    - attempt_respawn_reconnection_：是否尝试重新连接。
    - system_active_：系统是否处于活动状态。
    - diagnostics_updater_：诊断更新器。
    - bond_respawn_start_time_：绑定重生开始时间。
    - bond_respawn_max_duration_：绑定重生最大持续时间。
  */
  // Timer thread to look at bond connections
  rclcpp::TimerBase::SharedPtr init_timer_;  // 初始化定时器，用于检查节点的连接情况
  rclcpp::TimerBase::SharedPtr bond_timer_;  // 绑定定时器，用于检查绑定状态
  rclcpp::TimerBase::SharedPtr bond_respawn_timer_;  // 绑定重生定时器，用于在绑定超时后重新连接
  std::chrono::milliseconds bond_timeout_;  // 绑定超时时间

  // A map of all nodes to check bond connection
  // 所有节点的绑定映射表
  std::map<std::string, std::shared_ptr<bond::Bond>> bond_map_;

  // A map of all nodes to be controlled
  // 所有节点的生命周期服务客户端映射表
  std::map<std::string, std::shared_ptr<nav2_util::LifecycleServiceClient>> node_map_;

  // 状态转换标签映射表
  std::map<std::uint8_t, std::string> transition_label_map_;

  // A map of the expected transitions to primary states
  // 预期的状态转换到主状态的映射表
  std::unordered_map<std::uint8_t, std::uint8_t> transition_state_map_;

  // The names of the nodes to be managed, in the order of desired bring-up
  // 要管理的节点名称，按照期望的启动顺序排列
  std::vector<std::string> node_names_;

  // Whether to automatically start up the system
  bool autostart_;                                   // 是否自动启动系统
  bool attempt_respawn_reconnection_;                // 是否尝试重新连接

  bool system_active_{false};                        // 系统是否处于活动状态
  diagnostic_updater::Updater diagnostics_updater_;  // 诊断更新器

  rclcpp::Time bond_respawn_start_time_{0};          // 绑定重生开始时间
  rclcpp::Duration bond_respawn_max_duration_{10s};  // 绑定重生最大持续时间
};

}  // namespace nav2_lifecycle_manager

#endif  // NAV2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
