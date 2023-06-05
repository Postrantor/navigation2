// Copyright (c) 2020 Aitor Miguel Blanco
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_UPDATED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_UPDATED_CONDITION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when goal is
 * updated on the blackboard and FAILURE otherwise
 *
 * @details 这是一个继承自BT::ConditionNode的类，当目标在黑板上更新时返回SUCCESS，否则返回FAILURE
 *          This is a class inherited from BT::ConditionNode, which returns SUCCESS when the goal is updated on the blackboard, and FAILURE otherwise.
 */
class GoalUpdatedCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::GoalUpdatedCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   *
   * @details 构造函数，用于创建nav2_behavior_tree::GoalUpdatedCondition实例
   *          Constructor for creating an instance of nav2_behavior_tree::GoalUpdatedCondition
   */
  GoalUpdatedCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

  // 禁用默认构造函数
  // Disabling the default constructor
  GoalUpdatedCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   *
   * @details 重写tick()方法，这是BT行为所需要的主要方法
   *          Overriding the tick() method, which is the main method required by a BT action
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   *
   * @details 创建包含节点特定端口的BT端口列表
   *          Creates a BT PortsList containing node-specific ports
   */
  static BT::PortsList providedPorts() { return {}; }

private:
  // 存储目标状态的变量
  // Variable to store the goal state
  geometry_msgs::msg::PoseStamped goal_;
  // 存储多个目标状态的向量
  // Vector to store multiple goal states
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_UPDATED_CONDITION_HPP_
