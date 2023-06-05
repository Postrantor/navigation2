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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PLANNER_SELECTOR_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PLANNER_SELECTOR_NODE_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 规划器选择器行为用于切换规划器服务器将使用的规划器。它订阅一个名为 "planner_selector" 的主题，
 * 以获取有关必须使用哪个规划器的决策。通常在 ComputePathToPoseAction 之前使用。
 * selected_planner 输出端口传递给 ComputePathToPoseAction 的 planner_id 输入端口。
 * 
 * @brief The PlannerSelector behavior is used to switch the planner
 * that will be used by the planner server. It subscribes to a topic "planner_selector"
 * to get the decision about what planner must be used. It is usually used before of
 * the ComputePathToPoseAction. The selected_planner output port is passed to planner_id
 * input port of the ComputePathToPoseAction.
 */
class PlannerSelector : public BT::SyncActionNode
{
public:
  /**
   * @brief nav2_behavior_tree::PlannerSelector 的构造函数
   * @brief A constructor for nav2_behavior_tree::PlannerSelector
   *
   * @param xml_tag_name 该节点的 XML 标签名称
   * @param conf  BT 节点配置
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf  BT node configuration
   */
  PlannerSelector(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  /**
   * @brief 创建 BT 端口列表
   * @brief Creates list of BT ports
   * @return BT::PortsList 包含基本端口和特定于节点的端口
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "default_planner", "如果没有收到任何外部主题消息，则使用的默认规划器。"),
      // "the default planner to use if there is not any external topic message received."),

      BT::InputPort<std::string>("topic_name", "planner_selector", "选择规划器的输入主题名称"),
      // "the input topic name to select the planner"),

      BT::OutputPort<std::string>("selected_planner", "通过订阅选择的规划器")};
    // "Selected planner by subscription")};
  }

private:
  /**
   * @brief 在 tick 上执行一些用户定义的操作的函数
   * @brief Function to perform some user-defined operation on tick
   */
  BT::NodeStatus tick() override;

  /**
   * @brief planner_selector 主题的回调函数
   * @brief callback function for the planner_selector topic
   *
   * @param msg 带有 planner_selector id 的消息
   * @param msg the message with the id of the planner_selector
   */
  void callbackPlannerSelect(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_selector_sub_;

  std::string last_selected_planner_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string topic_name_;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PLANNER_SELECTOR_NODE_HPP_
