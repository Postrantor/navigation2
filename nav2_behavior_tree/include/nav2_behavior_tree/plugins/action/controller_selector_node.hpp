// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Pablo Iñigo Blasco
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONTROLLER_SELECTOR_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONTROLLER_SELECTOR_NODE_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief ControllerSelector行为用于切换控制器服务器将使用的控制器。它订阅一个名为"controller_selector"
 * 的主题以获取有关必须使用哪个控制器的决策。它通常在FollowPath之前使用。selected_controller输出端口传递给
 * FollowPath的controller_id输入端口
 * (The ControllerSelector behavior is used to switch the controller
 * that will be used by the controller server. It subscribes to a topic "controller_selector"
 * to get the decision about what controller must be used. It is usually used before of
 * the FollowPath. The selected_controller output port is passed to controller_id
 * input port of the FollowPath)
 */
class ControllerSelector : public BT::SyncActionNode
{
public:
  /**
   * @brief nav2_behavior_tree::ControllerSelector的构造函数
   * (A constructor for nav2_behavior_tree::ControllerSelector)
   *
   * @param xml_tag_name 此节点的XML标签名称 (Name for the XML tag for this node)
   * @param conf  BT节点配置 (BT node configuration)
   */
  ControllerSelector(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  /**
   * @brief 创建BT端口列表
   * (Creates list of BT ports)
   * @return 包含基本端口和节点特定端口的BT::PortsList
   * (BT::PortsList Containing basic ports along with node-specific ports)
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "default_controller",
        "如果没有收到任何外部主题消息，则使用默认控制器"
        "the default controller to use if there is not any external topic message received."),

      BT::InputPort<std::string>(
        "topic_name", "controller_selector",
        "用于选择控制器的输入主题名称"
        "the input topic name to select the controller"),

      BT::OutputPort<std::string>(
        "selected_controller", "通过订阅选择的控制器"
                               "Selected controller by subscription")};
  }

private:
  /**
   * @brief 在tick上执行一些用户定义的操作的函数
   * (Function to perform some user-defined operation on tick)
   */
  BT::NodeStatus tick() override;

  /**
   * @brief controller_selector主题的回调函数
   * (callback function for the controller_selector topic)
   *
   * @param msg 带有controller_selector id的消息
   * (the message with the id of the controller_selector)
   */
  void callbackControllerSelect(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_selector_sub_;

  std::string last_selected_controller_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string topic_name_;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONTROLLER_SELECTOR_NODE_HPP_
