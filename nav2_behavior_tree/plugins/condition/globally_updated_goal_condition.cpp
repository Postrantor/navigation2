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

#include <vector>
#include <string>

#include "nav2_behavior_tree/plugins/condition/globally_updated_goal_condition.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 构造函数，初始化 GloballyUpdatedGoalCondition 类的实例 (Constructor, initializes an instance of the GloballyUpdatedGoalCondition class)
 * 
 * @param condition_name 条件节点的名称 (The name of the condition node)
 * @param conf 节点配置 (Node configuration)
 */
GloballyUpdatedGoalCondition::GloballyUpdatedGoalCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf), // 调用基类 ConditionNode 的构造函数 (Call the base class ConditionNode's constructor)
  first_time(true) // 初始化 first_time 标志为 true (Initialize the first_time flag as true)
{
  // 获取 "node" 对应的共享指针，并存储到 node_ 成员变量中 (Get the shared pointer corresponding to "node" and store it in the node_ member variable)
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

/**
 * @brief tick 函数，执行条件节点的逻辑 (Tick function, execute the logic of the condition node)
 * 
 * @return BT::NodeStatus 返回节点状态 (Return the node status)
 */
BT::NodeStatus GloballyUpdatedGoalCondition::tick()
{
  if (first_time) { // 如果是第一次执行 (If it is the first time execution)
    first_time = false; // 将 first_time 标志设置为 false (Set the first_time flag to false)
    // 从黑板中获取 "goals" 和 "goal" 的值，并存储到 goals_ 和 goal_ 成员变量中 (Get the values of "goals" and "goal" from the blackboard and store them in the goals_ and goal_ member variables)
    config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", goals_);
    config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", goal_);
    return BT::NodeStatus::SUCCESS; // 返回成功状态 (Return success status)
  }

  // 定义 current_goals 和 current_goal 变量，用于存储当前的目标值 (Define current_goals and current_goal variables to store the current goal values)
  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  geometry_msgs::msg::PoseStamped current_goal;
  
  // 从黑板中获取 "goals" 和 "goal" 的当前值，并存储到 current_goals 和 current_goal 变量中 (Get the current values of "goals" and "goal" from the blackboard and store them in the current_goals and current_goal variables)
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  // 如果 goal_ 不等于 current_goal 或者 goals_ 不等于 current_goals (If goal_ is not equal to current_goal or goals_ is not equal to current_goals)
  if (goal_ != current_goal || goals_ != current_goals) {
    goal_ = current_goal; // 更新 goal_ 成员变量的值 (Update the value of the goal_ member variable)
    goals_ = current_goals; // 更新 goals_ 成员变量的值 (Update the value of the goals_ member variable)
    return BT::NodeStatus::SUCCESS; // 返回成功状态 (Return success status)
  }

  return BT::NodeStatus::FAILURE; // 如果没有更新目标值，则返回失败状态 (If the goal value is not updated, return failure status)
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GloballyUpdatedGoalCondition>("GlobalUpdatedGoal");
}
