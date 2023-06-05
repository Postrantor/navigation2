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

#include <sstream>
#include <stdexcept>
#include <string>

#include "nav2_behavior_tree/plugins/control/pipeline_sequence.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 构造函数，创建一个 PipelineSequence 实例。
 * @param name 节点的名称。
 *
 * @brief Constructor, creates a PipelineSequence instance.
 * @param name The name of the node.
 */
PipelineSequence::PipelineSequence(const std::string & name) : BT::ControlNode(name, {}) {}

/**
 * @brief 带配置的构造函数，创建一个 PipelineSequence 实例。
 * @param name 节点的名称。
 * @param config 节点的配置。
 *
 * @brief Constructor with configuration, creates a PipelineSequence instance.
 * @param name The name of the node.
 * @param config The configuration of the node.
 */
PipelineSequence::PipelineSequence(const std::string & name, const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

/**
 * @brief 执行 tick 函数。
 * @return 返回当前节点的状态。
 *
 * @brief Execute the tick function.
 * @return Returns the current status of the node.
 */
BT::NodeStatus PipelineSequence::tick()
{
  // 遍历所有子节点
  // Iterate through all child nodes
  for (std::size_t i = 0; i < children_nodes_.size(); ++i) {
    // 获取子节点的执行状态
    // Get the execution status of the child node
    auto status = children_nodes_[i]->executeTick();

    switch (status) {
    case BT::NodeStatus::FAILURE:
      // 如果子节点失败，则停止所有子节点并重置 last_child_ticked_
      // If a child node fails, stop all child nodes and reset last_child_ticked_
      ControlNode::haltChildren();
      last_child_ticked_ = 0; // reset
      return status;
    case BT::NodeStatus::SUCCESS:
      // 如果子节点成功，继续执行下一个子节点。如果是最后一个子节点，将退出循环并执行方法末尾的总结代码。
      // If a child node succeeds, continue to the next child node. If it is the last child node,
      // exit the loop and execute the wrap-up code at the end of the method.
      break;
    case BT::NodeStatus::RUNNING:
      if (i >= last_child_ticked_) {
        // 更新 last_child_ticked_ 并返回当前状态
        // Update last_child_ticked_ and return the current status
        last_child_ticked_ = i;
        return status;
      }
      // 否则，继续执行下一个子节点
      // Otherwise, continue to the next child node
      break;
    default:
      // 如果收到无效的节点状态，则抛出异常
      // Throw an exception if an invalid node status is received
      std::stringstream error_msg;
      error_msg << "Invalid node status. Received status " << status << "from child "
                << children_nodes_[i]->name();
      throw std::runtime_error(error_msg.str());
    }
  }
  // 总结代码。停止所有子节点并重置 last_child_ticked_
  // Wrap up code. Stop all child nodes and reset last_child_ticked_
  ControlNode::haltChildren();
  last_child_ticked_ = 0; // reset
  return BT::NodeStatus::SUCCESS;
}

/**
 * @brief 停止当前节点。
 *
 * @brief Halt the current node.
 */
void PipelineSequence::halt()
{
  // 调用父类的 halt 方法并重置 last_child_ticked_
  // Call the parent class's halt method and reset last_child_ticked_
  BT::ControlNode::halt();
  last_child_ticked_ = 0;
}

} // namespace nav2_behavior_tree

// 导入所需的头文件 (Include the necessary header files)
#include "nav2_behavior_tree/plugins/control/pipeline_sequence.hpp"

/**
 * @brief 注册节点类型 (Register node types)
 * 
 * @param factory 工厂对象，用于注册自定义行为树节点类型 (Factory object for registering custom behavior tree node types)
 */
BT_REGISTER_NODES(factory)
{
  // 使用工厂对象注册 nav2_behavior_tree::PipelineSequence 节点类型，并将其命名为 "PipelineSequence"
  // Register the nav2_behavior_tree::PipelineSequence node type with the factory object and name it "PipelineSequence"
  factory.registerNodeType<nav2_behavior_tree::PipelineSequence>("PipelineSequence");
}
