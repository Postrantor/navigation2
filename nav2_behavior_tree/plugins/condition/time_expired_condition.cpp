// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#include "nav2_behavior_tree/plugins/condition/time_expired_condition.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 构造一个 TimeExpiredCondition 类实例 (Constructs an instance of the TimeExpiredCondition class)
 *
 * @param condition_name 条件节点的名称 (The name of the condition node)
 * @param conf 节点配置 (Node configuration)
 */
TimeExpiredCondition::TimeExpiredCondition(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf), period_(1.0)
{
  // 从输入中获取 "seconds" 参数并设置 period_ 值 (Get the "seconds" parameter from the input and set the value of period_)
  getInput("seconds", period_);

  // 从黑板中获取 rclcpp::Node::SharedPtr 类型的 "node" 对象 (Get the "node" object of type rclcpp::Node::SharedPtr from the blackboard)
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // 设置开始时间为当前时间 (Set the start time to the current time)
  start_ = node_->now();
}

/**
 * @brief TimeExpiredCondition 的 tick 函数，用于检查条件是否满足 (Tick function for TimeExpiredCondition, used to check if the condition is met)
 *
 * @return BT::NodeStatus 返回节点状态 (Returns the node status)
 */
BT::NodeStatus TimeExpiredCondition::tick()
{
  // 如果节点状态为 IDLE，则重置开始时间并返回 FAILURE (If the node status is IDLE, reset the start time and return FAILURE)
  if (status() == BT::NodeStatus::IDLE) {
    start_ = node_->now();
    return BT::NodeStatus::FAILURE;
  }

  // 计算自开始时间以来经过的时间 (Calculate the time elapsed since the start time)
  auto elapsed = node_->now() - start_;

  // 将经过的时间转换为秒数 (Convert the elapsed time to seconds)
  auto seconds = elapsed.seconds();

  // 如果经过的秒数小于设定的周期，则返回 FAILURE (If the elapsed seconds are less than the set period, return FAILURE)
  if (seconds < period_) {
    return BT::NodeStatus::FAILURE;
  }

  // 重置开始时间 (Reset the start time)
  start_ = node_->now();

  // 返回 SUCCESS (Return SUCCESS)
  return BT::NodeStatus::SUCCESS;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TimeExpiredCondition>("TimeExpired");
}
