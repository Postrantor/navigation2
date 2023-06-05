// Copyright (c) 2020 Samsung Research America
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
#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/condition/transform_available_condition.hpp"

using namespace std::chrono_literals; // NOLINT

namespace nav2_behavior_tree
{

/**
 * @brief TransformAvailableCondition 构造函数
 * @param condition_name 条件节点名称
 * @param conf 节点配置
 *
 * @brief TransformAvailableCondition constructor
 * @param condition_name Name of the condition node
 * @param conf Node configuration
 */
TransformAvailableCondition::TransformAvailableCondition(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  was_found_(false) // 初始化列表，继承自 BT::ConditionNode 类，并初始化 was_found_ 为 false
{
  // 从黑板中获取 "node" 对应的值，并将其赋给 node_
  // Get the value corresponding to "node" from the blackboard and assign it to node_
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // 从黑板中获取 "tf_buffer" 对应的值，并将其赋给 tf_
  // Get the value corresponding to "tf_buffer" from the blackboard and assign it to tf_
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  // 获取输入参数 "child" 并赋值给 child_frame_
  // Get input parameter "child" and assign it to child_frame_
  getInput("child", child_frame_);

  // 获取输入参数 "parent" 并赋值给 parent_frame_
  // Get input parameter "parent" and assign it to parent_frame_

  getInput("parent", parent_frame_);

  // 如果 child_frame_ 或 parent_frame_ 为空，则打印错误信息并退出
  // If child_frame_ or parent_frame_ is empty, print an error message and exit
  if (child_frame_.empty() || parent_frame_.empty()) {
    RCLCPP_FATAL(
      node_->get_logger(), "Child frame (%s) or parent frame (%s) were empty.",
      child_frame_.c_str(), parent_frame_.c_str());
    exit(-1);
  }

  // 输出调试信息，表示已初始化 TransformAvailableCondition BT 节点
  // Print debug information indicating that the TransformAvailableCondition BT node has been initialized
  RCLCPP_DEBUG(node_->get_logger(), "Initialized an TransformAvailableCondition BT node");
}

/**
 * @brief TransformAvailableCondition 析构函数
 *
 * @brief TransformAvailableCondition destructor
 */
TransformAvailableCondition::~TransformAvailableCondition()
{
  // 输出调试信息，表示正在关闭 TransformAvailableCondition BT 节点
  // Print debug information indicating that the TransformAvailableCondition BT node is shutting down
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down TransformAvailableCondition BT node");
}

/**
 * @brief tick 函数，用于检查变换是否可用
 * @return 返回 BT::NodeStatus，表示成功或失败
 *
 * @brief tick function, used to check if the transform is available
 * @return Returns BT::NodeStatus, indicating success or failure
 */
BT::NodeStatus TransformAvailableCondition::tick()
{
  // 如果之前已经找到变换，则直接返回成功状态
  // If the transform was found before, return the success status directly
  if (was_found_) {
    return BT::NodeStatus::SUCCESS;
  }

  std::string tf_error;
  // 检查 child_frame_ 到 parent_frame_ 的变换是否可用，如果可用，将结果赋值给 found
  // Check if the transform from child_frame_ to parent_frame_ is available, and if so, assign the result to found
  bool found = tf_->canTransform(child_frame_, parent_frame_, tf2::TimePointZero, &tf_error);

  // 如果找到了变换
  // If the transform is found
  if (found) {
    was_found_ = true;
    return BT::NodeStatus::SUCCESS;
  }

  // 输出信息，表示从 child_frame_ 到 parent_frame_ 的变换未找到，并显示 tf 错误
  // Print information indicating that the transform from child_frame_ to parent_frame_ was not found, and display the tf error
  RCLCPP_INFO(
    node_->get_logger(), "Transform from %s to %s was not found, tf error: %s",
    child_frame_.c_str(), parent_frame_.c_str(), tf_error.c_str());

  // 返回失败状态
  // Return failure status
  return BT::NodeStatus::FAILURE;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TransformAvailableCondition>("TransformAvailable");
}
