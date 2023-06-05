// Copyright (c) 2021 Joshua Wallace
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GLOBALLY_UPDATED_GOAL_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GLOBALLY_UPDATED_GOAL_CONDITION_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{
/*!
 * @brief 一个 BT::ConditionNode，当目标在黑板上更新时返回 SUCCESS，否则返回 FAILURE
 *        A BT::ConditionNode that returns SUCCESS when goal is
 *        updated on the blackboard and FAILURE otherwise
 */
class GloballyUpdatedGoalCondition : public BT::ConditionNode
{
public:
  /*!
   * @brief nav2_behavior_tree::GloballyUpdatedGoalCondition 的构造函数
   *        A constructor for nav2_behavior_tree::GloballyUpdatedGoalCondition
   * @param condition_name 此节点的 XML 标签名称
   *                       Name for the XML tag for this node
   * @param conf BT 节点配置
   *             BT node configuration
   */
  GloballyUpdatedGoalCondition(
    const std::string & condition_name, const BT::NodeConfiguration & conf);

  // 删除默认构造函数
  // Delete default constructor
  GloballyUpdatedGoalCondition() = delete;

  /*!
   * @brief 主要覆盖 BT 操作所需的方法
   *        The main override required by a BT action
   * @return BT::NodeStatus 执行 tick 的状态
   *         BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /*!
   * @brief 创建 BT 端口列表
   *        Creates list of BT ports
   * @return BT::PortsList 包含节点特定端口的列表
   *         BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts() { return {}; }

private:
  // 表示是否是第一次执行的布尔变量
  // Boolean variable indicating if it's the first execution
  bool first_time;
  // rclcpp 节点的共享指针
  // Shared pointer to an rclcpp node
  rclcpp::Node::SharedPtr node_;
  // 目标位置的消息类型变量
  // Message type variable for the goal position
  geometry_msgs::msg::PoseStamped goal_;
  // 包含多个目标位置的向量
  // Vector containing multiple goal positions
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GLOBALLY_UPDATED_GOAL_CONDITION_HPP_
