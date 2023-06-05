// Copyright (c) 2021 Samsung Research America
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

#include <chrono>
#include <string>

#include "nav2_behavior_tree/plugins/decorator/single_trigger_node.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 构造函数，用于初始化 SingleTrigger 类的实例
 *        Constructor for initializing an instance of the SingleTrigger class
 *
 * @param name 节点的名称
 *        Name of the node
 * @param conf 节点的配置
 *        Configuration of the node
 */
SingleTrigger::SingleTrigger(const std::string & name, const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf), first_time_(true)
{
  // 设置 first_time_ 为 true，表示这是第一次触发
  // Set first_time_ to true, indicating this is the first trigger
}

/**
 * @brief tick() 方法，用于执行节点的逻辑
 *        The tick() method, used to execute the logic of the node
 *
 * @return 返回节点状态 (NodeStatus)
 *         Returns the node status (NodeStatus)
 */
BT::NodeStatus SingleTrigger::tick()
{
  // 如果当前节点状态为 IDLE
  // If the current node status is IDLE
  if (status() == BT::NodeStatus::IDLE) {
    first_time_ = true;
  }

  // 将节点状态设置为 RUNNING
  // Set the node status to RUNNING
  setStatus(BT::NodeStatus::RUNNING);

  // 如果是第一次触发
  // If it's the first trigger
  if (first_time_) {
    // 执行子节点的 tick() 方法，并获取其状态
    // Execute the tick() method of the child node and get its status
    const BT::NodeStatus child_state = child_node_->executeTick();

    // 根据子节点的状态进行相应的处理
    // Handle accordingly based on the status of the child node
    switch (child_state) {
    case BT::NodeStatus::RUNNING:
      // 如果子节点状态为 RUNNING，则返回 RUNNING
      // If the child node status is RUNNING, return RUNNING
      return BT::NodeStatus::RUNNING;

    case BT::NodeStatus::SUCCESS:
      // 如果子节点状态为 SUCCESS，设置 first_time_ 为 false，并返回 SUCCESS
      // If the child node status is SUCCESS, set first_time_ to false and return SUCCESS
      first_time_ = false;
      return BT::NodeStatus::SUCCESS;

    case BT::NodeStatus::FAILURE:
      // 如果子节点状态为 FAILURE，设置 first_time_ 为 false，并返回 FAILURE
      // If the child node status is FAILURE, set first_time_ to false and return FAILURE
      first_time_ = false;
      return BT::NodeStatus::FAILURE;

    default:
      // 默认情况下，设置 first_time_ 为 false，并返回 FAILURE
      // By default, set first_time_ to false and return FAILURE
      first_time_ = false;
      return BT::NodeStatus::FAILURE;
    }
  }

  // 如果不是第一次触发，则直接返回 FAILURE
  // If it's not the first trigger, return FAILURE directly
  return BT::NodeStatus::FAILURE;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SingleTrigger>("SingleTrigger");
}
