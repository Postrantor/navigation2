// Copyright (c) 2022 Neobotix GmbH
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

#ifndef NAV2_BEHAVIOR_TREE__BT_CANCEL_ACTION_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__BT_CANCEL_ACTION_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace nav2_behavior_tree {

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief Abstract class representing an action for cancelling BT node
 * @tparam ActionT Type of action
 */
template <class ActionT>
class BtCancelActionNode : public BT::ActionNodeBase {
public:
  /**
   * @brief A nav2_behavior_tree::BtCancelActionNode constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  BtCancelActionNode(
      const std::string& xml_tag_name,
      const std::string& action_name,
      const BT::NodeConfiguration& conf)
      // 初始化列表（Initializer list）
      : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name) {
    // 从黑板中获取节点指针
    // Get node pointer from the blackboard
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");

    // 创建互斥的回调组
    // Create a mutually exclusive callback group
    callback_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

    // 将回调组添加到执行器中
    // Add the callback group to the executor
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // 从黑板中获取所需项目
    // Get the required items from the blackboard
    server_timeout_ =
        config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

    // 获取输入的服务器名称，并重新映射动作名称
    // Get input server name and remap the action name
    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      action_name_ = remapped_action_name;
    }

    // 创建动作客户端
    // Create the action client
    createActionClient(action_name_);

    // 给派生类一个做任何初始化的机会
    // Give the derived class a chance to do any initialization
    RCLCPP_DEBUG(
        node_->get_logger(), "\"%s\" BtCancelActionNode initialized", xml_tag_name.c_str());
  }

  // 删除默认构造函数
  // Delete the default constructor
  BtCancelActionNode() = delete;

  // 虚拟析构函数
  // Virtual destructor
  virtual ~BtCancelActionNode() {}

  /**
   * @brief 创建一个动作客户端实例 (Create instance of an action client)
   * @param action_name 要为其创建客户端的动作名称 (Action name to create client for)
   */
  void createActionClient(const std::string& action_name) {
    // 现在我们有了要使用的 ROS 节点，为此 BT 动作创建动作客户端
    // (Now that we have the ROS node to use, create the action client for this BT action)
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name, callback_group_);

    // 在继续之前确保服务器确实存在 (Make sure the server is actually there before continuing)
    RCLCPP_DEBUG(
        node_->get_logger(), "等待 \"%s\" 动作服务器 (Waiting for \"%s\" action server)",
        action_name.c_str());
    if (!action_client_->wait_for_action_server(1s)) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "\"%s\" 动作服务器在等待 1 秒后仍然不可用 (\"%s\" action server not available after "
          "waiting for 1 s)",
          action_name.c_str());
      throw std::runtime_error(
          std::string("动作服务器 ") + action_name + std::string(" 不可用 (Action server ") +
          action_name + std::string(" not available"));
    }
  }

  /**
   * @brief 接受参数的 BtCancelActionNode 的任何子类都必须提供 providedPorts 方法并在其中调用
   * providedBasicPorts。 (Any subclass of BtCancelActionNode that accepts parameters must provide a
   *         providedPorts method and call providedBasicPorts in it.)
   * @param addition 要添加到 BT 端口列表中的附加端口 (Additional ports to add to BT port list)
   * @return BT::PortsList 包含基本端口以及节点特定端口的列表 (BT::PortsList Containing basic ports
   * along with node-specific ports)
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition) {
    BT::PortsList basic = {
        BT::InputPort<std::string>("server_name", "动作服务器名称 (Action server name)"),
        BT::InputPort<std::chrono::milliseconds>("server_timeout")};
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  // 停止当前操作 (Halt the current operation)
  void halt() {}

protected:
  // 动作名称 (Action name)
  std::string action_name_;
  // 动作客户端，使用 std::shared_ptr 管理其生命周期 (Action client, using std::shared_ptr to manage
  // its lifetime)
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;
  // 将用于任何 ROS 操作的节点 (The node that will be used for any ROS operations)
  rclcpp::Node::SharedPtr node_;
  // 回调组，负责处理回调函数 (Callback group, responsible for handling callback functions)
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  // 单线程执行器，用于执行回调组中的回调函数 (Single-threaded executor, used to execute callback
  // functions in the callback group)
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  // 当新的动作目标被取消时，等待服务器响应的超时值 (The timeout value while waiting for response
  // from a server when a new action goal is canceled)
  std::chrono::milliseconds server_timeout_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_CANCEL_ACTION_NODE_HPP_
