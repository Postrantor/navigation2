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

#include "nav2_lifecycle_manager/lifecycle_manager.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using nav2_util::LifecycleServiceClient;

namespace nav2_lifecycle_manager {

/**
 * @brief LifecycleManager 构造函数
 * @param options rclcpp::NodeOptions 类型，ROS2 节点选项
 * @details
 *   1. 初始化 Node 对象，名称为 "lifecycle_manager"，使用传入的 options 参数
 *   2. 声明并初始化一些 ROS2 节点参数，包括：
 *      - node_names: 字符串数组类型，节点名列表
 *      - autostart: bool 类型，是否自动启动节点
 *      - bond_timeout: double 类型，连接超时时间
 *      - bond_respawn_max_duration: double 类型，重连最大持续时间
 *      - attempt_respawn_reconnection: bool 类型，是否尝试重新连接
 *   3. 注册 rclcpp::shutdown() 回调函数
 *   4. 获取节点参数值，并进行类型转换
 *   5. 创建回调组和服务对象
 *   6. 初始化状态转换映射表和标签映射表
 *   7. 创建定时器，用于初始化生命周期服务客户端、启动节点、创建线程等操作
 *   8. 创建诊断更新器，用于生成节点健康状态信息
 *
 * @details
 *   构造函数初始化了参数，并创建了两个服务，一个是ManageLifecycleNodes，另一个是std_srvs::srv::Trigger。
 *   还创建了一个定时器init_timer_，在回调函数中执行了createLifecycleServiceClients()和startup()方法。
 *   最后创建了一个线程service_thread_，并将callback_group_添加到executor中。
 *   同时还创建了一个diagnostics_updater_对象，用于更新Nav2的健康状态。
 */
LifecycleManager::LifecycleManager(const rclcpp::NodeOptions& options)
    : Node("lifecycle_manager", options), diagnostics_updater_(this) {
  RCLCPP_INFO(get_logger(), "Creating");

  // 参数化节点名称列表，允许该模块与不同的节点集一起使用
  declare_parameter("node_names", rclcpp::PARAMETER_STRING_ARRAY);
  declare_parameter("autostart", rclcpp::ParameterValue(false));
  declare_parameter("bond_timeout", 4.0);
  declare_parameter("bond_respawn_max_duration", 10.0);
  declare_parameter("attempt_respawn_reconnection", true);

  registerRclPreshutdownCallback();

  node_names_ = get_parameter("node_names").as_string_array();
  get_parameter("autostart", autostart_);
  double bond_timeout_s;
  get_parameter("bond_timeout", bond_timeout_s);
  bond_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(bond_timeout_s));

  double respawn_timeout_s;
  get_parameter("bond_respawn_max_duration", respawn_timeout_s);
  bond_respawn_max_duration_ = rclcpp::Duration::from_seconds(respawn_timeout_s);

  get_parameter("attempt_respawn_reconnection", attempt_respawn_reconnection_);

  // 创建一个回调组callback_group_
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  // 创建ManageLifecycleNodes服务
  manager_srv_ = create_service<ManageLifecycleNodes>(
      get_name() + std::string("/manage_nodes"),
      std::bind(&LifecycleManager::managerCallback, this, _1, _2, _3), rclcpp::SystemDefaultsQoS(),
      callback_group_);
  // 创建is_active服务
  is_active_srv_ = create_service<std_srvs::srv::Trigger>(
      get_name() + std::string("/is_active"),
      std::bind(&LifecycleManager::isActiveCallback, this, _1, _2, _3), rclcpp::SystemDefaultsQoS(),
      callback_group_);

  // 初始化转换状态映射表和标签映射表
  transition_state_map_[Transition::TRANSITION_CONFIGURE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_CLEANUP] = State::PRIMARY_STATE_UNCONFIGURED;
  transition_state_map_[Transition::TRANSITION_ACTIVATE] = State::PRIMARY_STATE_ACTIVE;
  transition_state_map_[Transition::TRANSITION_DEACTIVATE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
      State::PRIMARY_STATE_FINALIZED;

  transition_label_map_[Transition::TRANSITION_CONFIGURE] = std::string("Configuring ");
  transition_label_map_[Transition::TRANSITION_CLEANUP] = std::string("Cleaning up ");
  transition_label_map_[Transition::TRANSITION_ACTIVATE] = std::string("Activating ");
  transition_label_map_[Transition::TRANSITION_DEACTIVATE] = std::string("Deactivating ");
  transition_label_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
      std::string("Shutting down ");

  // 创建一个定时器init_timer_
  init_timer_ = this->create_wall_timer(0s, [this]() -> void {
    init_timer_->cancel();
    createLifecycleServiceClients();  // 创建生命周期服务客户端
    if (autostart_) {
      init_timer_ = this->create_wall_timer(
          0s,
          [this]() -> void {
            init_timer_->cancel();
            startup();  // 启动节点
          },
          callback_group_);
    }
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_callback_group(callback_group_, get_node_base_interface());
    service_thread_ = std::make_unique<nav2_util::NodeThread>(executor);  // 创建线程service_thread_
  });
  diagnostics_updater_.setHardwareID("Nav2");                             // 设置硬件ID
  diagnostics_updater_.add(
      "Nav2 Health", this, &LifecycleManager::CreateActiveDiagnostic);  // 添加健康状态更新函数
}

/**
 * @brief LifecycleManager 析构函数
 * @details 销毁生命周期管理器对象，并输出日志信息
 */
LifecycleManager::~LifecycleManager() {
  RCLCPP_INFO(get_logger(), "Destroying %s", get_name());
  service_thread_.reset();
}

/*
  上述代码是ROS2项目中navigation2组件中的Lifecycle Manager功能相关的代码。
  其中包含两个函数：managerCallback和isActiveCallback。

  managerCallback函数根据请求中的命令信息，调用对应的生命周期管理函数，并将执行结果写入响应中。具体而言，该函数接受三个参数：请求头指针、请求指针和响应指针。请求指针是ManageLifecycleNodes::Request类型的指针，包含命令信息；响应指针是ManageLifecycleNodes::Response类型的指针，包含执行结果。在函数内部，使用switch语句根据请求中的命令信息，分别调用startup、reset、shutdown、pause和resume等五个生命周期管理函数，并将执行结果写入响应中。

  isActiveCallback函数用于获取系统是否激活的信息，并将其写入响应中。具体而言，该函数接受三个参数：请求头指针、请求指针和响应指针。请求指针是std_srvs::srv::Trigger::Request类型的指针，未使用；响应指针是std_srvs::srv::Trigger::Response类型的指针，包含系统是否激活的信息。在函数内部，将系统是否激活的信息写入响应中。
*/

/**
 * @brief LifecycleManager组件的managerCallback函数
 * @param request_header 请求头指针，未使用
 * @param request ManageLifecycleNodes::Request类型的请求指针，包含命令信息
 * @param response ManageLifecycleNodes::Response类型的响应指针，包含执行结果
 * @details 根据请求中的命令信息，调用对应的生命周期管理函数，并将执行结果写入响应中
 */
void LifecycleManager::managerCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<ManageLifecycleNodes::Request> request,
    std::shared_ptr<ManageLifecycleNodes::Response> response) {
  switch (request->command) {
    case ManageLifecycleNodes::Request::STARTUP:
      response->success = startup();
      break;
    case ManageLifecycleNodes::Request::RESET:
      response->success = reset();
      break;
    case ManageLifecycleNodes::Request::SHUTDOWN:
      response->success = shutdown();
      break;
    case ManageLifecycleNodes::Request::PAUSE:
      response->success = pause();
      break;
    case ManageLifecycleNodes::Request::RESUME:
      response->success = resume();
      break;
  }
}

/**
 * @brief LifecycleManager组件的isActiveCallback函数
 * @param request_header 请求头指针，未使用
 * @param request std_srvs::srv::Trigger::Request类型的请求指针，未使用
 * @param response std_srvs::srv::Trigger::Response类型的响应指针，包含系统是否激活的信息
 * @details 将系统是否激活的信息写入响应中
 */
void LifecycleManager::isActiveCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  response->success = system_active_;
}

/*
这段代码是 navigation2 组件中 lifecycle_manager 功能相关的代码。其中，CreateActiveDiagnostic
函数用于创建并返回一个包含当前系统状态的诊断信息；createLifecycleServiceClients
函数用于创建和初始化生命周期服务客户端；destroyLifecycleServiceClients
函数用于销毁生命周期服务客户端。在 createLifecycleServiceClients 和 destroyLifecycleServiceClients
函数中，都使用了节点映射来存储生命周期服务客户端，以便于管理和操作。
*/

/**
 * @brief 创建并返回一个包含当前系统状态的诊断信息
 * @param stat 诊断信息对象
 * @details 如果系统处于活动状态，将返回“Nav2 is active”，否则返回“Nav2 is inactive”
 */
void LifecycleManager::CreateActiveDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  if (system_active_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Nav2 is active");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Nav2 is inactive");
  }
}

/**
 * @brief 创建和初始化生命周期服务客户端
 * @details 遍历节点名称列表，为每个节点创建一个生命周期服务客户端，并将其添加到节点映射中
 */
void LifecycleManager::createLifecycleServiceClients() {
  message("Creating and initializing lifecycle service clients");
  for (auto& node_name : node_names_) {
    node_map_[node_name] = std::make_shared<LifecycleServiceClient>(node_name, shared_from_this());
  }
}

/**
 * @brief 销毁生命周期服务客户端
 * @details 遍历节点映射，逐一销毁生命周期服务客户端
 */
void LifecycleManager::destroyLifecycleServiceClients() {
  message("Destroying lifecycle service clients");
  for (auto& kv : node_map_) {
    kv.second.reset();
  }
}

/*
该函数用于创建节点间的bond连接，具体实现如下：

- 首先计算出timeout_ns和timeout_s，分别表示超时时间的纳秒数和秒数。
-
如果bond_map_中不存在该节点名称，且bond_timeout_的值大于0，则创建一个新的Bond对象，并将其存储在bond_map_中。
- 然后设置心跳超时时间为timeout_s，心跳周期为0.10秒，并启动Bond连接。
- 如果连接成功，则返回true，否则返回false。如果连接失败，则输出错误信息并返回false。
*/

/**
 * @brief 创建节点间的bond连接
 * @param node_name 节点名称
 * @details
 * 如果bond_map_中不存在该节点名称，且bond_timeout_的值大于0，则创建一个新的Bond对象，并将其存储在bond_map_中。
 *          然后设置心跳超时时间为timeout_s，心跳周期为0.10秒，并启动Bond连接。如果连接成功，则返回true，否则返回false。
 *          如果连接失败，则输出错误信息并返回false。
 */
bool LifecycleManager::createBondConnection(const std::string& node_name) {
  // 计算超时时间的纳秒数
  const double timeout_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(bond_timeout_).count();
  // 将超时时间转换为秒数
  const double timeout_s = timeout_ns / 1e9;

  // 如果bond_map_中不存在该节点名称，且bond_timeout_的值大于0，则创建一个新的Bond对象，并将其存储在bond_map_中。
  if (bond_map_.find(node_name) == bond_map_.end() && bond_timeout_.count() > 0.0) {
    // 创建一个新的Bond对象，并将其存储在bond_map_中
    bond_map_[node_name] = std::make_shared<bond::Bond>("bond", node_name, shared_from_this());
    // 设置心跳超时时间为timeout_s
    bond_map_[node_name]->setHeartbeatTimeout(timeout_s);
    // 设置心跳周期为0.10秒
    bond_map_[node_name]->setHeartbeatPeriod(0.10);
    // 启动Bond连接
    bond_map_[node_name]->start();

    // 如果连接成功，则返回true，否则返回false。
    if (!bond_map_[node_name]->waitUntilFormed(
            rclcpp::Duration(rclcpp::Duration::from_nanoseconds(timeout_ns / 2)))) {
      // 如果连接失败，则输出错误信息并返回false。
      RCLCPP_ERROR(
          get_logger(),
          "Server %s was unable to be reached after %0.2fs by bond. "
          "This server may be misconfigured.",
          node_name.c_str(), timeout_s);
      return false;
    }
    // 输出连接成功的信息
    RCLCPP_INFO(get_logger(), "Server %s connected with bond.", node_name.c_str());
  }

  return true;
}

/*
该代码段是在 ROS2 项目中 Navigation2 组件的 Lifecycle Manager 功能相关的代码。LifecycleManager 类是
Navigation2 中的一个重要组件，用于管理节点的生命周期状态。changeStateForNode 函数是 LifecycleManager
类的一个公共函数，用于改变指定节点的生命周期状态。

该函数首先调用 message
函数打印出正在执行的生命周期转换和节点名称。然后检查是否能够改变指定节点的生命周期状态，如果不能，则打印错误信息并返回
false。如果可以改变节点的生命周期状态，则根据生命周期转换标识符执行不同的操作：如果是激活转换，则创建一个新的
Bond 连接；如果是停用转换，则从 Bond 映射中删除节点名称。最后返回 true
表示成功改变了节点的生命周期状态。
*/

/**
 * @brief LifecycleManager::changeStateForNode 函数用于改变指定节点的生命周期状态。
 * @param node_name 节点名称。
 * @param transition 生命周期转换标识符。
 * @return 如果成功改变了节点的生命周期状态，则返回 true，否则返回 false。
 * @details 该函数首先调用 message
 * 函数打印出正在执行的生命周期转换和节点名称。然后检查是否能够改变指定节点的生命周期状态，如果不能，则打印错误信息并返回
 * false。如果可以改变节点的生命周期状态，则根据生命周期转换标识符执行不同的操作：如果是激活转换，则创建一个新的
 * Bond 连接；如果是停用转换，则从 Bond 映射中删除节点名称。最后返回 true
 * 表示成功改变了节点的生命周期状态。
 */
bool LifecycleManager::changeStateForNode(const std::string& node_name, std::uint8_t transition) {
  message(transition_label_map_[transition] + node_name);

  // 检查是否能够改变指定节点的生命周期状态
  if (!node_map_[node_name]->change_state(transition) ||
      !(node_map_[node_name]->get_state() == transition_state_map_[transition])) {
    RCLCPP_ERROR(get_logger(), "Failed to change state for node: %s", node_name.c_str());
    return false;
  }

  // 根据生命周期转换标识符执行不同的操作
  if (transition == Transition::TRANSITION_ACTIVATE) {
    return createBondConnection(node_name);
  } else if (transition == Transition::TRANSITION_DEACTIVATE) {
    bond_map_.erase(node_name);
  }

  return true;
}

/*
功能总结：该函数实现了对一组节点进行生命周期状态转换的操作。通过遍历所有节点，并调用
changeStateForNode 函数来改变每个节点的生命周期状态。如果某个节点改变失败并且不是强制改变，则返回
false；如果所有节点都成功改变状态，则返回 true。

其中，hard_change 参数表示是否强制改变，即使某个节点失败了也会继续执行。transition
参数表示生命周期状态转换类型，包括 TRANSITION_CONFIGURE 和 TRANSITION_ACTIVATE
两种情况。如果是前者，则按照正常顺序遍历所有节点；如果是后者，则按照相反的顺序遍历所有节点。在遍历过程中，如果某个节点改变失败，则根据
hard_change 参数决定是否继续执行下一个节点。如果所有节点都成功改变状态，则返回 true。
*/

/**
 * @brief 改变所有节点的生命周期状态
 * @param transition 生命周期状态转换类型
 * @param hard_change 是否强制改变，即使某个节点失败了也会继续执行
 * @details 遍历所有节点，对每个节点调用 changeStateForNode 函数改变其生命周期状态
 *          如果某个节点改变失败并且不是强制改变，则返回 false
 *          如果某个节点改变失败但是是强制改变，则继续执行下一个节点
 *          如果所有节点都成功改变状态，则返回 true
 */
bool LifecycleManager::changeStateForAllNodes(std::uint8_t transition, bool hard_change) {
  // 强制改变选项将继续执行即使某个节点失败
  if (transition == Transition::TRANSITION_CONFIGURE ||
      transition == Transition::TRANSITION_ACTIVATE) {
    // 遍历所有节点，按照顺序依次改变其生命周期状态
    for (auto& node_name : node_names_) {
      try {
        // 调用 changeStateForNode 函数改变节点生命周期状态
        // 如果节点改变失败并且不是强制改变，则返回 false
        if (!changeStateForNode(node_name, transition) && !hard_change) {
          return false;
        }
      } catch (const std::runtime_error& e) {
        // 输出错误信息
        RCLCPP_ERROR(
            get_logger(), "Failed to change state for node: %s. Exception: %s.", node_name.c_str(),
            e.what());
        return false;
      }
    }
  } else {
    // 反向遍历所有节点，按照相反的顺序依次改变其生命周期状态
    std::vector<std::string>::reverse_iterator rit;
    for (rit = node_names_.rbegin(); rit != node_names_.rend(); ++rit) {
      try {
        // 调用 changeStateForNode 函数改变节点生命周期状态
        // 如果节点改变失败并且不是强制改变，则返回 false
        if (!changeStateForNode(*rit, transition) && !hard_change) {
          return false;
        }
      } catch (const std::runtime_error& e) {
        // 输出错误信息
        RCLCPP_ERROR(
            get_logger(), "Failed to change state for node: %s. Exception: %s.", (*rit).c_str(),
            e.what());
        return false;
      }
    }
  }
  return true;
}

/*
其中包含了三个函数：shutdownAllNodes()、startup()和shutdown()。

shutdownAllNodes()函数通过调用三个不同的状态转换函数，对所有节点进行关闭操作。
startup()函数则是对所有被管理的节点进行配置和激活操作。如果有任何一个节点无法完成这两个操作，则返回false。如果所有节点都成功启动，则返回true。
最后，shutdown()函数对所有被管理的节点进行关闭操作，并销毁生命周期服务客户端。如果所有节点都成功关闭，则返回true。
*/

/**
 * @brief 关闭所有节点
 * @param 无
 * @details 使用三个不同的状态转换函数对所有节点进行关闭操作
 */
void LifecycleManager::shutdownAllNodes() {
  message("Deactivate, cleanup, and shutdown nodes");
  changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE);
  changeStateForAllNodes(Transition::TRANSITION_CLEANUP);
  changeStateForAllNodes(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
}

/**
 * @brief 启动所有被管理的节点
 * @param 无
 * @return bool 返回启动是否成功
 * @details
 * 对所有被管理的节点进行配置和激活操作，如果有任何一个节点无法完成这两个操作，则返回false。
 *          如果所有节点都成功启动，则返回true。
 */
bool LifecycleManager::startup() {
  message("Starting managed nodes bringup...");
  if (!changeStateForAllNodes(Transition::TRANSITION_CONFIGURE) ||
      !changeStateForAllNodes(Transition::TRANSITION_ACTIVATE)) {
    RCLCPP_ERROR(get_logger(), "Failed to bring up all requested nodes. Aborting bringup.");
    return false;
  }
  message("Managed nodes are active");
  system_active_ = true;
  createBondTimer();
  return true;
}

/**
 * @brief 关闭所有被管理的节点
 * @param 无
 * @return bool 返回关闭是否成功
 * @details
 * 对所有被管理的节点进行关闭操作，并销毁生命周期服务客户端。如果所有节点都成功关闭，则返回true。
 */
bool LifecycleManager::shutdown() {
  system_active_ = false;
  destroyBondTimer();

  message("Shutting down managed nodes...");
  shutdownAllNodes();
  destroyLifecycleServiceClients();
  message("Managed nodes have been shut down");
  return true;
}

/*
这段代码是在ROS2项目中navigation2组件中lifecycle_manager功能相关的代码。其中包含了两个函数reset和pause，用于重置和暂停lifecycle
manager中的所有节点状态。

reset函数首先将system_active_设置为false，销毁bond
timer，然后对所有的节点进行反向转换（即从activate到deactivate，再到cleanup）。如果在转换过程中出现错误，则根据hard_reset参数判断是否继续执行。如果不是硬重置，则输出错误信息并返回false。最后输出“Managed
nodes have been reset”并返回true。

pause函数将system_active_设置为false，销毁bond
timer，然后对所有的节点进行deactivate操作。如果在转换过程中出现错误，则输出错误信息并返回false。最后输出“Managed
nodes have been paused”并返回true。
*/

/**
 * @brief 重置lifecycle manager中的所有节点状态
 * @param hard_reset 是否进行硬重置
 * @details
 * 首先将system_active_设置为false，销毁bond timer。
 * 然后对所有的节点进行反向转换（即从activate到deactivate，再到cleanup）。
 * 如果在转换过程中出现错误，则根据hard_reset参数判断是否继续执行。
 * 如果不是硬重置，则输出错误信息并返回false。
 * 最后输出“Managed nodes have been reset”并返回true。
 */
bool LifecycleManager::reset(bool hard_reset) {
  system_active_ = false;
  destroyBondTimer();

  message("Resetting managed nodes...");
  // Should transition in reverse order
  if (!changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE, hard_reset) ||
      !changeStateForAllNodes(Transition::TRANSITION_CLEANUP, hard_reset)) {
    if (!hard_reset) {
      RCLCPP_ERROR(get_logger(), "Failed to reset nodes: aborting reset");
      return false;
    }
  }

  message("Managed nodes have been reset");
  return true;
}

/**
 * @brief 暂停lifecycle manager中的所有节点状态
 * @details
 * 将system_active_设置为false，销毁bond timer。
 * 然后对所有的节点进行deactivate操作。
 * 如果在转换过程中出现错误，则输出错误信息并返回false。
 * 最后输出“Managed nodes have been paused”并返回true。
 */
bool LifecycleManager::pause() {
  system_active_ = false;
  destroyBondTimer();

  message("Pausing managed nodes...");
  if (!changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE)) {
    RCLCPP_ERROR(get_logger(), "Failed to pause nodes: aborting pause");
    return false;
  }

  message("Managed nodes have been paused");
  return true;
}

/*
代码段的功能是在 navigation2 组件中 lifecycle_manager 功能相关的代码。其中 LifecycleManager
类是一个节点管理器，用于管理所有被它所创建的节点。resume()
函数用于恢复所有被管理的节点，即对所有被管理的节点执行激活操作，如果有任何一个节点无法激活，则返回
false；否则将系统状态设置为激活状态，并创建 bond 定时器。createBondTimer() 函数用于创建 bond
定时器，如果 bond 超时时间小于等于 0，则直接返回；否则创建定时器，每隔 200ms 执行一次
checkBondConnections 函数。destroyBondTimer() 函数用于销毁 bond 定时器，如果 bond
定时器存在，则取消定时器并将其重置为空。
*/

/**
 * @brief LifecycleManager::resume() 恢复所有被管理的节点
 * @details 对所有被管理的节点执行激活操作，如果操作失败，则返回 false。
 *
 * @return true 如果所有节点都成功激活
 * @return false 如果有任何一个节点无法激活
 */
bool LifecycleManager::resume() {
  message("Resuming managed nodes...");  // 输出日志信息，表示正在恢复被管理的节点
  if (!changeStateForAllNodes(Transition::TRANSITION_ACTIVATE)) {  // 对所有被管理的节点执行激活操作
    RCLCPP_ERROR(
        get_logger(),
        "Failed to resume nodes: aborting resume");  // 如果有任何一个节点无法激活，则输出错误日志信息并返回
                                                     // false
    return false;
  }

  message("Managed nodes are active");  // 输出日志信息，表示所有被管理的节点已经激活
  system_active_ = true;                // 将系统状态设置为激活状态
  createBondTimer();                    // 创建 bond 定时器
  return true;                          // 返回 true 表示所有节点都已经成功激活
}

/**
 * @brief LifecycleManager::createBondTimer() 创建 bond 定时器
 * @details 如果 bond 超时时间小于等于 0，则直接返回；否则创建定时器，每隔 200ms 执行一次
 * checkBondConnections 函数。
 */
void LifecycleManager::createBondTimer() {
  if (bond_timeout_.count() <= 0) {  // 如果 bond 超时时间小于等于 0，则直接返回
    return;
  }

  message("Creating bond timer...");  // 输出日志信息，表示正在创建 bond 定时器
  bond_timer_ = this->create_wall_timer(
      200ms, std::bind(&LifecycleManager::checkBondConnections, this),
      callback_group_);  // 创建定时器，每隔 200ms 执行一次 checkBondConnections 函数
}

/**
 * @brief LifecycleManager::destroyBondTimer() 销毁 bond 定时器
 * @details 如果 bond 定时器存在，则取消定时器并将其重置为空。
 */
void LifecycleManager::destroyBondTimer() {
  if (bond_timer_) {                       // 如果 bond 定时器存在
    message("Terminating bond timer...");  // 输出日志信息，表示正在销毁 bond 定时器
    bond_timer_->cancel();                 // 取消定时器
    bond_timer_.reset();                   // 将 bond 定时器重置为空
  }
}

/*
这段代码是在ROS2项目中navigation2组件中lifecycle_manager功能相关的代码。其中包含了两个函数：onRclPreshutdown()和registerRclPreshutdownCallback()。

onRclPreshutdown()函数是在ROS2系统即将关闭时，LifecycleManager组件会调用该函数进行清理工作。具体来说，它会销毁bond定时器，并清空节点名称列表、节点映射表和bond映射表。

registerRclPreshutdownCallback()函数则是注册LifecycleManager组件的onRclPreshutdown回调函数，当ROS2系统即将关闭时会被调用。在函数中，它会添加pre-shutdown回调函数，以便在ROS2系统即将关闭时执行相应的清理工作。
*/

/**
 * @brief LifecycleManager类的onRclPreshutdown函数
 * @details 当ROS2系统即将关闭时，LifecycleManager组件会调用该函数进行清理工作。
 */
void LifecycleManager::onRclPreshutdown() {
  RCLCPP_INFO(get_logger(), "Running Nav2 LifecycleManager rcl preshutdown (%s)", this->get_name());

  destroyBondTimer();

  /*
   * Dropping the bond map is what we really need here, but we drop the others
   * to prevent the bond map being used. Likewise, squash the service thread.
   */
  // 销毁bond定时器
  service_thread_.reset();
  // 清空节点名称列表
  node_names_.clear();
  // 清空节点映射表
  node_map_.clear();
  // 清空bond映射表
  bond_map_.clear();
}

/**
 * @brief LifecycleManager类的registerRclPreshutdownCallback函数
 * @details 注册LifecycleManager组件的onRclPreshutdown回调函数，当ROS2系统即将关闭时会被调用。
 */
void LifecycleManager::registerRclPreshutdownCallback() {
  rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();

  // 添加pre-shutdown回调函数
  context->add_pre_shutdown_callback(std::bind(&LifecycleManager::onRclPreshutdown, this));
}

/*
该代码段是 navigation2 组件中 lifecycle_manager 功能相关的代码。该函数用于检查所有节点与
LifecycleManager 的 bond 连接状态，如果有节点连接失败，则关闭所有相关节点，并初始化 bond
重连定时器以便在最大超时时间内检查服务器是否重新上线。

具体实现上，首先判断系统是否激活、rclcpp::ok() 是否返回 false 或 bond_map
是否为空，如果是则直接返回。然后遍历所有节点，检查其 bond 连接状态，如果某个节点的 bond
连接已断开，则进行处理：关闭所有相关节点、清空 bond_map、初始化 bond 重连定时器。
*/

/**
 * @brief 检查所有节点与 LifecycleManager 的 bond 连接状态
 * @details 如果系统未激活、rclcpp::ok() 返回 false 或 bond_map 为空，则直接返回。
 */
void LifecycleManager::checkBondConnections() {
  if (!system_active_ || !rclcpp::ok() || bond_map_.empty()) {
    return;
  }

  // 遍历所有节点，检查其 bond 连接状态
  for (auto& node_name : node_names_) {
    if (!rclcpp::ok()) {
      return;
    }

    // 如果某个节点的 bond 连接已断开，则进行处理
    if (bond_map_[node_name]->isBroken()) {
      message(std::string("Have not received a heartbeat from " + node_name + "."));

      // 如果有一个节点连接失败，则关闭所有相关节点
      RCLCPP_ERROR(
          get_logger(),
          "CRITICAL FAILURE: SERVER %s IS DOWN after not receiving a heartbeat for %i ms."
          " Shutting down related nodes.",
          node_name.c_str(), static_cast<int>(bond_timeout_.count()));
      reset(true);  // hard reset to transition all still active down
      // if a server crashed, it won't get cleared due to failed transition, clear manually
      bond_map_.clear();

      // 初始化 bond 重连定时器，以便在最大超时时间内检查服务器是否重新上线
      if (attempt_respawn_reconnection_) {
        bond_respawn_timer_ = this->create_wall_timer(
            1s, std::bind(&LifecycleManager::checkBondRespawnConnection, this), callback_group_);
      }
      return;
    }
  }
}

/*
功能梳理：该函数用于检查重新连接的绑定。首先判断是否是第一次尝试重新生成，如果是，则启动重新生成的最大持续时间；然后检查绑定失败后的活动连接数，如果所有连接都处于活动状态，则杀死计时器并将系统重新转换为活动状态；否则，检查是否已经达到了最大超时时间，如果是，则输出失败信息。
*/

/**
 * @brief 检查重新连接的绑定
 * @param 无
 * @details 如果在重新生成时第一次尝试，则启动重新生成的最大持续时间。
 *          注意：system_active_被反转，因为这应该是一个失败条件。如果另一个外部用户再次激活系统，则不应处理。
 *          检查绑定失败后的活动连接数。如果所有连接都处于活动状态，请杀死计时器并将系统重新转换为活动状态。否则，检查是否已经达到了最大超时时间。
 */
void LifecycleManager::checkBondRespawnConnection() {
  // 如果在重新生成时第一次尝试，则启动重新生成的最大持续时间。
  if (bond_respawn_start_time_.nanoseconds() == 0) {
    bond_respawn_start_time_ = now();
  }

  // 注意：system_active_被反转，因为这应该是一个失败条件。如果另一个外部用户再次激活系统，则不应处理。
  if (system_active_ || !rclcpp::ok() || node_names_.empty()) {
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
    return;
  }

  // 检查绑定失败后的活动连接数
  int live_servers = 0;
  const int max_live_servers = node_names_.size();
  for (auto& node_name : node_names_) {
    if (!rclcpp::ok()) {
      return;
    }

    // 只有当服务器存在时才不会抛出异常
    try {
      node_map_[node_name]->get_state();
      live_servers++;
    } catch (...) {
      break;
    }
  }

  // 如果所有连接都处于活动状态，请杀死计时器并将系统重新转换为活动状态。否则，检查是否已经达到了最大超时时间。
  if (live_servers == max_live_servers) {
    message("成功重新建立服务器重生的连接，重新启动。");
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
    startup();
  } else if (now() - bond_respawn_start_time_ >= bond_respawn_max_duration_) {
    message("在最大超时时间后无法从服务器崩溃中重新建立连接。");
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
  }
}

#define ANSI_COLOR_RESET "\x1b[0m"
#define ANSI_COLOR_BLUE "\x1b[34m"

void LifecycleManager::message(const std::string& msg) {
  RCLCPP_INFO(get_logger(), ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());
}

}  // namespace nav2_lifecycle_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_lifecycle_manager::LifecycleManager)
