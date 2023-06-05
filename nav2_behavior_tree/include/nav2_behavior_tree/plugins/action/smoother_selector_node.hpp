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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SMOOTHER_SELECTOR_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SMOOTHER_SELECTOR_NODE_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief The SmootherSelector behavior is used to switch the smoother
 * that will be used by the smoother server. It subscribes to a topic "smoother_selector"
 * to get the decision about what smoother must be used. It is usually used before of
 * the FollowPath. The selected_smoother output port is passed to smoother_id
 * input port of the FollowPath
 *
 * 平滑选择器行为用于切换平滑服务器使用的平滑器。它订阅了一个名为 "smoother_selector" 的主题，
 * 以获取关于应使用哪个平滑器的决策。它通常在 FollowPath 之前使用。
 * selected_smoother 输出端口传递给 FollowPath 的 smoother_id 输入端口。
 */
class SmootherSelector : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::SmootherSelector
   *
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf  BT node configuration
   *
   * @brief nav2_behavior_tree::SmootherSelector 的构造函数
   *
   * @param xml_tag_name 此节点的 XML 标签名称
   * @param conf  BT 节点配置
   */
  SmootherSelector(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   *
   * @brief 创建 BT 端口列表
   * @return BT::PortsList 包含基本端口以及节点特定端口
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "default_smoother",
        // the default smoother to use if there is not any external topic message received.
        "如果没有收到任何外部主题消息，则使用默认的平滑器。"),

      BT::InputPort<std::string>(
        "topic_name", "smoother_selector",
        // the input topic name to select the smoother
        "选择平滑器的输入主题名称"),

      BT::OutputPort<std::string>(
        "selected_smoother",
        // Selected smoother by subscription
        "通过订阅选择的平滑器")};
  }

private:
  /**
   * @brief Function to perform some user-defined operation on tick
   *
   * @brief 在 tick 上执行某些用户定义的操作的函数
   */
  BT::NodeStatus tick() override;

  /**
   * @brief callback function for the smoother_selector topic
   *
   * @param msg the message with the id of the smoother_selector
   *
   * @brief 平滑选择器主题的回调函数
   *
   * @param msg 带有平滑选择器 ID 的消息
   */
  void callbackSmootherSelect(const std_msgs::msg::String::SharedPtr msg);

  // Subscription object for the smoother_selector topic
  // 平滑选择器主题的订阅对象
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr smoother_selector_sub_;

  // Last selected smoother from the subscription
  // 订阅中最后选择的平滑器
  std::string last_selected_smoother_;

  // Node shared pointer
  // 节点共享指针
  rclcpp::Node::SharedPtr node_;

  // Callback group shared pointer
  // 回调组共享指针
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Single threaded executor for the callback group
  // 回调组的单线程执行器
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // Topic name for smoother_selector
  // 平滑选择器的主题名称
  std::string topic_name_;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SMOOTHER_SELECTOR_NODE_HPP_
