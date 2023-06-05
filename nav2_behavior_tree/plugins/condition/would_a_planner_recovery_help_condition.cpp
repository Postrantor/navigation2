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

#include "nav2_behavior_tree/plugins/condition/would_a_planner_recovery_help_condition.hpp"
#include <memory>

namespace nav2_behavior_tree
{

/**
 * @brief 构造函数，用于创建一个 WouldAPlannerRecoveryHelp 对象。
 *        Constructor for creating a WouldAPlannerRecoveryHelp object.
 *
 * @param condition_name 条件节点的名称。Name of the condition node.
 * @param conf 行为树节点配置。Behavior tree node configuration.
 */
WouldAPlannerRecoveryHelp::WouldAPlannerRecoveryHelp(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: AreErrorCodesPresent(condition_name, conf) // 调用基类构造函数。Calling base class constructor.
{
  // 初始化要检查的错误代码列表。Initialize the list of error codes to check.
  error_codes_to_check_ = {
    ActionGoal::UNKNOWN,       // 未知错误。Unknown error.
    ActionGoal::NO_VALID_PATH, // 没有有效路径。No valid path.
    ActionGoal::TIMEOUT        // 超时。Timeout.
  };
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::WouldAPlannerRecoveryHelp>(
    "WouldAPlannerRecoveryHelp");
}
