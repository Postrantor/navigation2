// Copyright (c) 2022 Joshua Wallace
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

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"

#include "nav2_behavior_tree/plugins/condition/path_expiring_timer_condition.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 构造函数，初始化 PathExpiringTimerCondition 类的实例 (Constructor, initializes an instance of the PathExpiringTimerCondition class)
 * @param condition_name 条件节点的名称 (Name of the condition node)
 * @param conf 节点配置 (Node configuration)
 */
PathExpiringTimerCondition::PathExpiringTimerCondition(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf), period_(1.0), first_time_(true)
{
  // 获取输入参数 "seconds" 的值 (Get the value of input parameter "seconds")
  getInput("seconds", period_);
  
  // 从黑板中获取共享指针类型的 rclcpp::Node 实例 (Get the shared pointer type rclcpp::Node instance from the blackboard)
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

/**
 * @brief tick 函数，用于执行条件节点的逻辑 (Tick function, used to execute the logic of the condition node)
 * @return 返回节点状态 (Returns the node status)
 */
BT::NodeStatus PathExpiringTimerCondition::tick()
{
  // 如果是第一次执行 tick 函数 (If it's the first time executing the tick function)
  if (first_time_) {
    // 获取输入参数 "path" 的值 (Get the value of input parameter "path")
    getInput("path", prev_path_);
    
    // 标记已经执行过 tick 函数 (Mark that the tick function has been executed)
    first_time_ = false;
    
    // 记录当前时间 (Record the current time)
    start_ = node_->now();
    
    // 返回 FAILURE 状态 (Return FAILURE status)
    return BT::NodeStatus::FAILURE;
  }

  // 获取新路径 (Grab the new path)
  nav_msgs::msg::Path path;
  getInput("path", path);

  // 如果路径已更新，重置计时器 (Reset timer if the path has been updated)
  if (prev_path_ != path) {
    prev_path_ = path;
    start_ = node_->now();
  }

  // 计算自迭代开始以来经过的时间 (Determine how long it's been since we've started this iteration)
  auto elapsed = node_->now() - start_;

  // 将经过的时间转换为秒 (Now, get that in seconds)
  auto seconds = elapsed.seconds();

  // 如果经过的时间小于设定的周期 (If the elapsed time is less than the set period)
  if (seconds < period_) {
    // 返回 FAILURE 状态 (Return FAILURE status)
    return BT::NodeStatus::FAILURE;
  }

  // 重置计时器 (Reset the timer)
  start_ = node_->now();
  
  // 返回 SUCCESS 状态 (Return SUCCESS status)
  return BT::NodeStatus::SUCCESS;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathExpiringTimerCondition>("PathExpiringTimer");
}
