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

#include "nav2_behavior_tree/plugins/condition/initial_pose_received_condition.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 判断是否接收到初始位置 (Check if the initial pose has been received)
 *
 * @param tree_node 传入的行为树节点，用于获取黑板数据 (The behavior tree node used to get blackboard data)
 * @return 返回节点状态，成功表示已接收到初始位置，失败表示未接收到 (Returns the node status, SUCCESS indicates that the initial pose has been received, FAILURE indicates that it has not been received)
 */
BT::NodeStatus initialPoseReceived(BT::TreeNode & tree_node)
{
  // 从行为树节点的黑板中获取 "initial_pose_received" 这个布尔值，若为真则说明已接收到初始位置
  // Get the boolean value "initial_pose_received" from the blackboard of the behavior tree node. If it is true, it means that the initial pose has been received.
  auto initPoseReceived = tree_node.config().blackboard->get<bool>("initial_pose_received");

  // 根据 initPoseReceived 的值返回相应的节点状态
  // Return the corresponding node status based on the value of initPoseReceived
  return initPoseReceived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerSimpleCondition(
    "InitialPoseReceived",
    std::bind(&nav2_behavior_tree::initialPoseReceived, std::placeholders::_1));
}
