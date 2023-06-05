// Copyright (c) 2019 Intel Corporation
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

#include <string>

#include "nav2_behavior_tree/plugins/control/round_robin_node.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 构造函数，用于创建一个 RoundRobinNode 实例 (Constructor for creating a RoundRobinNode instance)
 *
 * @param name 节点的名称 (Name of the node)
 */
RoundRobinNode::RoundRobinNode(const std::string & name) : BT::ControlNode::ControlNode(name, {}) {}

/**
 * @brief 带配置参数的构造函数，用于创建一个 RoundRobinNode 实例 (Constructor with configuration parameters for creating a RoundRobinNode instance)
 *
 * @param name 节点的名称 (Name of the node)
 * @param config 节点的配置参数 (Configuration parameters of the node)
 */
RoundRobinNode::RoundRobinNode(const std::string & name, const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

/**
 * @brief tick 函数，用于执行 RoundRobinNode 的逻辑 (Tick function for executing the logic of RoundRobinNode)
 *
 * @return 返回节点的状态 (Returns the status of the node)
 */
BT::NodeStatus RoundRobinNode::tick()
{
  // 获取子节点的数量 (Get the number of child nodes)
  const auto num_children = children_nodes_.size();

  // 设置节点的状态为运行中 (Set the status of the node to RUNNING)
  setStatus(BT::NodeStatus::RUNNING);

  // 当失败的子节点数量小于总子节点数量时，继续循环 (Continue looping while the number of failed child nodes is less than the total number of child nodes)
  while (num_failed_children_ < num_children) {
    // 获取当前子节点 (Get the current child node)
    TreeNode * child_node = children_nodes_[current_child_idx_];
    // 执行当前子节点的 tick 函数并获取状态 (Execute the tick function of the current child node and get its status)
    const BT::NodeStatus child_status = child_node->executeTick();

    // 根据子节点的状态进行相应的处理 (Handle the child node's status accordingly)
    switch (child_status) {
    case BT::NodeStatus::SUCCESS: {
      // 如果当前子节点索引已经到达最后一个子节点，则将其重置为第一个子节点 (Wrap around to the first child if the current child index has reached the last child node)
      if (++current_child_idx_ >= num_children) {
        current_child_idx_ = 0;
      }
      // 将失败子节点数量重置为 0 (Reset the number of failed child nodes to 0)
      num_failed_children_ = 0;
      // 停止其他子节点 (Halt other child nodes)
      ControlNode::haltChildren();
      // 返回成功状态 (Return SUCCESS status)
      return BT::NodeStatus::SUCCESS;
    }

    case BT::NodeStatus::FAILURE: {
      // 如果当前子节点索引已经到达最后一个子节点，则将其重置为第一个子节点 (Wrap around to the first child if the current child index has reached the last child node)
      if (++current_child_idx_ >= num_children) {
        current_child_idx_ = 0;
      }
      // 失败子节点数量加一 (Increment the number of failed child nodes)
      num_failed_children_++;
      break;
    }

    case BT::NodeStatus::RUNNING: {
      // 返回运行中状态 (Return RUNNING status)
      return BT::NodeStatus::RUNNING;
    }

    default: {
      // 如果返回了无效的状态，则抛出逻辑错误异常 (Throw a LogicError exception if an invalid status is returned)
      throw BT::LogicError("Invalid status return from BT node");
    }
    }
  }

  // 停止当前节点 (Halt the current node)
  halt();
  // 返回失败状态 (Return FAILURE status)
  return BT::NodeStatus::FAILURE;
}

/**
 * @brief 停止 RoundRobinNode 的执行 (Halt the execution of the RoundRobinNode)
 *
 * @param 无参数 (No parameters)
 * @return 无返回值 (No return value)
 */
void RoundRobinNode::halt()
{
  // 调用基类 ControlNode 的 halt() 方法，以停止当前节点 (Call the halt() method of the base class ControlNode to stop the current node)
  ControlNode::halt();

  // 将当前子节点索引重置为 0 (Reset the current child index to 0)
  current_child_idx_ = 0;

  // 将失败子节点数量重置为 0 (Reset the number of failed children to 0)
  num_failed_children_ = 0;
}

} // namespace nav2_behavior_tree

// 注册节点类型到节点工厂 (Register the node type to the node factory)
BT_REGISTER_NODES(factory)
{
  // 注册 RoundRobinNode 类型的节点，并将其命名为 "RoundRobin"
  // (Register the RoundRobinNode type node and name it "RoundRobin")
  factory.registerNodeType<nav2_behavior_tree::RoundRobinNode>("RoundRobin");
}