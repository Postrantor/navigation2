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

#include "nav2_behavior_tree/plugins/control/recovery_node.hpp"
#include <string>

namespace nav2_behavior_tree
{

/**
 * @brief 构造一个 RecoveryNode 对象 (Constructs a RecoveryNode object)
 * 
 * @param name 节点的名称 (The name of the node)
 * @param conf 节点配置 (The node configuration)
 */
RecoveryNode::RecoveryNode(const std::string & name, const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf), current_child_idx_(0), number_of_retries_(1),
  retry_count_(0)
{
  // 获取输入参数 "number_of_retries" 的值 (Get the value of input parameter "number_of_retries")
  getInput("number_of_retries", number_of_retries_);
}

/**
 * @brief 执行 RecoveryNode 的 tick 方法 (Executes the tick method of the RecoveryNode)
 * 
 * @return BT::NodeStatus 返回节点状态 (Returns the node status)
 */
BT::NodeStatus RecoveryNode::tick()
{
  // 获取子节点的数量 (Get the number of child nodes)
  const unsigned children_count = children_nodes_.size();

  // 检查子节点数量是否为 2，否则抛出异常 (Check if there are exactly 2 child nodes, otherwise throw an exception)
  if (children_count != 2) {
    throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
  }

  // 设置当前节点状态为 RUNNING (Set the current node status to RUNNING)
  setStatus(BT::NodeStatus::RUNNING);

  // 当前子节点索引小于子节点数量且重试次数小于等于允许的最大重试次数时，执行循环 (Execute the loop while the current child index is less than the number of child nodes and the retry count is less than or equal to the maximum allowed retries)
  while (current_child_idx_ < children_count && retry_count_ <= number_of_retries_) {
    // 获取当前子节点 (Get the current child node)
    TreeNode * child_node = children_nodes_[current_child_idx_];
    // 执行子节点的 tick 方法并获取返回的状态 (Execute the tick method of the child node and get the returned status)
    const BT::NodeStatus child_status = child_node->executeTick();

    // 如果当前子节点是第一个子节点 (If the current child node is the first child node)
    if (current_child_idx_ == 0) {
      switch (child_status) {
      case BT::NodeStatus::SUCCESS: {
        // 当第一个子节点返回成功时，重置节点并返回成功状态
        // (When the first child node returns success, reset the node and return the success status)
        halt();
        return BT::NodeStatus::SUCCESS;
      }

      case BT::NodeStatus::FAILURE: {
        if (retry_count_ < number_of_retries_) {
          // 当重试次数小于允许的最大重试次数时，停止第一个子节点并在下一次迭代中执行第二个子节点
          // (When the retry count is less than the maximum allowed retries, stop the first child node and execute the second child node in the next iteration)
          ControlNode::haltChild(0);
          current_child_idx_++;
          break;
        } else {
          // 当达到最大重试次数时，重置节点并返回失败状态
          // (When the maximum retries has been exceeded, reset the node and return the failure status)
          halt();
          return BT::NodeStatus::FAILURE;
        }
      }

      case BT::NodeStatus::RUNNING: {
        return BT::NodeStatus::RUNNING;
      }

      default: {
        throw BT::LogicError("A child node must never return IDLE");
      }
      } // end switch

    } else if (current_child_idx_ == 1) {
      switch (child_status) {
      case BT::NodeStatus::SUCCESS: {
        // 当第二个子节点返回成功时，停止第二个子节点，增加恢复计数，并在下一次迭代中执行第一个子节点
        // (When the second child node returns success, stop the second child node, increment the recovery count, and execute the first child node in the next iteration)
        ControlNode::haltChild(1);
        retry_count_++;
        current_child_idx_--;
      } break;

      case BT::NodeStatus::FAILURE: {
        // 当第二个子节点返回失败时，重置节点并返回失败状态
        // (When the second child node returns failure, reset the node and return the failure status)
        halt();
        return BT::NodeStatus::FAILURE;
      }

      case BT::NodeStatus::RUNNING: {
        return BT::NodeStatus::RUNNING;
      }

      default: {
        throw BT::LogicError("A child node must never return IDLE");
      }
      } // end switch
    }
  } // end while loop

  // 重置节点并返回失败状态 (Reset the node and return the failure status)
  halt();
  return BT::NodeStatus::FAILURE;
}

/**
 * @brief 停止 RecoveryNode 并重置相关变量 (Halts the RecoveryNode and resets related variables)
 */
void RecoveryNode::halt()
{
  // 调用 ControlNode 的 halt 方法 (Call the halt method of ControlNode)
  ControlNode::halt();
  // 重置重试计数和当前子节点索引 (Reset the retry count and current child index)
  retry_count_ = 0;
  current_child_idx_ = 0;
}

} // namespace nav2_behavior_tree

/**
 * @brief 这段代码主要用于将自定义的 RecoveryNode 类型注册到 Behavior Tree 的节点工厂中，以便在构建 Behavior Tree 时使用。
 */

// 引入 BehaviorTree 库中的 bt_factory.h 头文件
// Include the bt_factory.h header file from the BehaviorTree library
#include "behaviortree_cpp_v3/bt_factory.h"

/**
 * @brief 注册 Behavior Tree 节点到节点工厂
 * @param factory BehaviorTreeFactory 对象，用于注册自定义节点类型
 *
 * Register Behavior Tree nodes to the node factory
 * @param factory The BehaviorTreeFactory object used to register custom node types
 */
BT_REGISTER_NODES(factory)
{
  /**
   * @brief 使用节点工厂注册 RecoveryNode 类型，其名称为 "RecoveryNode"
   * @brief 使用 nav2_behavior_tree 命名空间
   *
   * Register the RecoveryNode type with the node factory, named "RecoveryNode"
   * Use the nav2_behavior_tree namespace
   */
  factory.registerNodeType<nav2_behavior_tree::RecoveryNode>("RecoveryNode");
}
