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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GOAL_CHECKER_SELECTOR_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GOAL_CHECKER_SELECTOR_NODE_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief GoalCheckerSelector行为用于切换控制器服务器的目标检查器。它订阅一个名为"goal_checker_selector"的主题，
 * 以获取有关必须使用哪个目标检查器的决策。它通常在FollowPath之前使用。selected_goal_checker输出端口传递给
 * FollowPath的goal_checker_id输入端口。
 * The GoalCheckerSelector behavior is used to switch the goal checker of the controller server. 
 * It subscribes to a topic "goal_checker_selector" to get the decision about what goal_checker must be used. 
 * It is usually used before of the FollowPath. The selected_goal_checker output port is passed to goal_checker_id
 * input port of the FollowPath.
 */
class GoalCheckerSelector : public BT::SyncActionNode
{
public:
  /**
   * @brief nav2_behavior_tree::GoalCheckerSelector的构造函数
   * A constructor for nav2_behavior_tree::GoalCheckerSelector
   *
   * @param xml_tag_name 此节点的XML标签名称 Name for the XML tag for this node
   * @param conf  BT节点配置 BT node configuration
   */
  GoalCheckerSelector(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  /**
   * @brief 创建BT端口列表 Creates list of BT ports
   * @return BT::PortsList 包含基本端口和节点特定端口 Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {// 默认的目标检查器，如果没有收到任何外部主题消息，则使用
            // the default goal_checker to use if there is not any external topic message received.
            BT::InputPort<std::string>("default_goal_checker"),

            // 用于选择目标检查器的输入主题名称
            // the input topic name to select the goal_checker
            BT::InputPort<std::string>("topic_name", "goal_checker_selector"),

            // 通过订阅选择的目标检查器
            // Selected goal_checker by subscription
            BT::OutputPort<std::string>("selected_goal_checker")};
  }

private:
  /**
   * @brief 执行某些用户定义操作的函数 Function to perform some user-defined operation on tick
   */
  BT::NodeStatus tick() override;

  /**
   * @brief goal_checker_selector主题的回调函数 callback function for the goal_checker_selector topic
   *
   * @param msg 带有goal_checker_selector id的消息 the message with the id of the goal_checker_selector
   */
  void callbackGoalCheckerSelect(const std_msgs::msg::String::SharedPtr msg);

  // 订阅goal_checker_selector的对象
  // Subscription object for goal_checker_selector
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_checker_selector_sub_;

  // 最后选择的目标检查器字符串
  // Last selected goal checker string
  std::string last_selected_goal_checker_;

  // ROS2节点共享指针
  // ROS2 node shared pointer
  rclcpp::Node::SharedPtr node_;

  // 主题名称字符串
  // Topic name string
  std::string topic_name_;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__GOAL_CHECKER_SELECTOR_NODE_HPP_
