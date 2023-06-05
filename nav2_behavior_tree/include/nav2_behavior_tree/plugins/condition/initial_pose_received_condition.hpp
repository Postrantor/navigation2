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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__INITIAL_POSE_RECEIVED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__INITIAL_POSE_RECEIVED_CONDITION_HPP_

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace nav2_behavior_tree
{
/**
 * @brief 一个 BT::ConditionNode，如果收到初始位置，则返回 SUCCESS；否则返回 FAILURE。
 *        (A BT::ConditionNode that returns SUCCESS if initial pose has been received and FAILURE otherwise.)
 *
 * @param tree_node 树节点引用，用于获取节点信息。(The tree node reference, used to get node information.)
 * @return 返回 BT::NodeStatus 状态，表示是否收到初始位置。(Returns BT::NodeStatus status, indicating whether the initial pose has been received.)
 */
BT::NodeStatus initialPoseReceived(BT::TreeNode & tree_node);
} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__INITIAL_POSE_RECEIVED_CONDITION_HPP_
