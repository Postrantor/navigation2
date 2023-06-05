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

#include "nav2_behavior_tree/plugins/condition/goal_updated_condition.hpp"
#include <string>
#include <vector>

namespace nav2_behavior_tree
{

/**
 * @brief 构造函数，初始化 GoalUpdatedCondition 类的对象 (Constructor, initializing an object of the GoalUpdatedCondition class)
 * 
 * @param condition_name 条件节点名称 (Condition node name)
 * @param conf 节点配置信息 (Node configuration information)
 */
GoalUpdatedCondition::GoalUpdatedCondition(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf) // 初始化 BT::ConditionNode 基类 (Initialize the BT::ConditionNode base class)
{
}

/**
 * @brief tick 函数，用于检查目标是否更新 (Tick function, used to check if the goal is updated)
 *
 * @return BT::NodeStatus 返回节点状态，表示目标是否更新 (Return node status, indicating whether the goal is updated or not)
 */
BT::NodeStatus GoalUpdatedCondition::tick()
{
  // 如果节点状态为 IDLE (If the node status is IDLE)
  if (status() == BT::NodeStatus::IDLE) {
    // 从黑板中获取 goals 和 goal 的值 (Get the values of goals and goal from the blackboard)
    config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", goals_);
    config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", goal_);
    return BT::NodeStatus::FAILURE; // 返回 FAILURE 状态 (Return FAILURE status)
  }

  // 定义一个当前目标列表和当前目标变量 (Define a current goals list and a current goal variable)
  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  geometry_msgs::msg::PoseStamped current_goal;

  // 从黑板中获取当前 goals 和 goal 的值 (Get the current values of goals and goal from the blackboard)
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  // 如果当前目标与之前的目标不同，或者当前目标列表与之前的目标列表不同
  // (If the current goal is different from the previous goal, or the current goals list is different from the previous goals list)
  if (goal_ != current_goal || goals_ != current_goals) {
    goal_ = current_goal;     // 更新目标 (Update the goal)
    goals_ = current_goals;   // 更新目标列表 (Update the goals list)
    return BT::NodeStatus::SUCCESS; // 返回 SUCCESS 状态 (Return SUCCESS status)
  }

  return BT::NodeStatus::FAILURE; // 返回 FAILURE 状态 (Return FAILURE status)
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalUpdatedCondition>("GoalUpdated");
}
