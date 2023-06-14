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

#include "nav2_util/lifecycle_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

namespace nav2_util {

/*
  其中，LifecycleNode类继承自rclcpp_lifecycle::LifecycleNode，实现了节点的生命周期管理功能。
  - 在构造函数中，设置了永不超时的参数，并注册了rcl_preshutdown_callback回调函数；
  - 在析构函数中，销毁节点并移除rcl_preshutdown_callback回调函数；
  - 在createBond函数中，创建bond连接到lifecycle manager，并设置心跳周期和超时时间。
*/

/**
 * @brief LifecycleNode类的构造函数
 * @param node_name 节点名称
 * @param ns 命名空间
 * @param options 节点选项
 * @details 构造函数继承自rclcpp_lifecycle::LifecycleNode，实现了节点的生命周期管理功能。
 */
LifecycleNode::LifecycleNode(
    const std::string& node_name,  //
    const std::string& ns,         //
    const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode(node_name, ns, options) {
  // server side never times out from lifecycle manager
  // 设置永不超时
  this->declare_parameter(bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true);
  this->set_parameter(
      rclcpp::Parameter(bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true));

  // 打印生命周期通知
  printLifecycleNodeNotification();

  // 注册rcl_preshutdown_callback回调函数
  register_rcl_preshutdown_callback();
}

/**
 * @brief LifecycleNode类的析构函数
 * @details 析构函数销毁节点并移除rcl_preshutdown_callback回调函数。
 */
LifecycleNode::~LifecycleNode() {
  RCLCPP_INFO(get_logger(), "Destroying");

  // 运行清理函数
  runCleanups();

  if (rcl_preshutdown_cb_handle_) {
    // 获取节点上下文
    rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();
    // 移除rcl_preshutdown_callback回调函数
    context->remove_pre_shutdown_callback(*(rcl_preshutdown_cb_handle_.get()));
    rcl_preshutdown_cb_handle_.reset();
  }
}

/**
 * @brief 创建bond连接到lifecycle manager
 * @details 创建bond连接到lifecycle manager，并设置心跳周期和超时时间。
 *
 * 这些支持设置的参数可以考虑
 */
void LifecycleNode::createBond() {
  RCLCPP_INFO(get_logger(), "Creating bond (%s) to lifecycle manager.", this->get_name());

  // 创建bond连接
  bond_ = std::make_unique<bond::Bond>(std::string("bond"), this->get_name(), shared_from_this());

  // 设置心跳周期和超时时间
  bond_->setHeartbeatPeriod(0.10);
  bond_->setHeartbeatTimeout(4.0);
  bond_->start();
}

/**
 * @brief 运行清理函数
 * @details 如果此生命周期节点没有被正确关闭，则在此处进行关闭。
 *         我们将为用户提供一些适当的清理能力，但这是最好的努力；即我们不会尝试考虑所有可能的状态。
 */
void LifecycleNode::runCleanups() {
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    this->deactivate();
  }

  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    this->cleanup();
  }
}

/**
 * @brief rcl preshutdown回调函数
 * @details 在rcl preshutdown时运行Nav2 LifecycleNode，用于清理和销毁bond
 */
void LifecycleNode::on_rcl_preshutdown() {
  RCLCPP_INFO(get_logger(), "Running Nav2 LifecycleNode rcl preshutdown (%s)", this->get_name());

  runCleanups();

  destroyBond();
}

/**
 * @brief 注册rcl preshutdown回调函数
 * @details 获取节点基础接口的上下文，并添加rcl preshutdown回调函数
 */
void LifecycleNode::register_rcl_preshutdown_callback() {
  rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();

  rcl_preshutdown_cb_handle_ = std::make_unique<rclcpp::PreShutdownCallbackHandle>(
      context->add_pre_shutdown_callback(std::bind(&LifecycleNode::on_rcl_preshutdown, this)));
}

/**
 * @brief 销毁bond
 * @details 销毁与lifecycle manager之间的bond
 */
void LifecycleNode::destroyBond() {
  RCLCPP_INFO(get_logger(), "Destroying bond (%s) to lifecycle manager.", this->get_name());

  if (bond_) {
    bond_.reset();
  }
}

/**
 * @brief 打印生命周期节点通知
 * @details 打印生命周期节点启动信息，等待外部生命周期转换以激活。
 *        更多信息请参见https://design.ros2.org/articles/node_lifecycle.html。
 */
void LifecycleNode::printLifecycleNodeNotification() {
  RCLCPP_INFO(
      get_logger(),
      "\n\t%s lifecycle node launched. \n"
      "\tWaiting on external lifecycle transitions to activate\n"
      "\tSee https://design.ros2.org/articles/node_lifecycle.html for more information.",
      get_name());
}

}  // namespace nav2_util
