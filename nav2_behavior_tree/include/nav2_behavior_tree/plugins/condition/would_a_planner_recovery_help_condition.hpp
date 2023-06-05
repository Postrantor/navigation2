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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__WOULD_A_PLANNER_RECOVERY_HELP_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__WOULD_A_PLANNER_RECOVERY_HELP_CONDITION_HPP_

#include <string>

#include "nav2_behavior_tree/plugins/condition/are_error_codes_present_condition.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

namespace nav2_behavior_tree
{

/**
 * @class WouldAPlannerRecoveryHelp
 * @brief 判断是否需要进行规划器恢复的类 (Class to determine if a planner recovery is needed)
 * @details 继承自 AreErrorCodesPresent 类 (Inherits from the AreErrorCodesPresent class)
 */
class WouldAPlannerRecoveryHelp : public AreErrorCodesPresent
{
  // 使用 nav2_msgs::action::ComputePathToPose 动作 (Use the nav2_msgs::action::ComputePathToPose action)
  using Action = nav2_msgs::action::ComputePathToPose;
  // 使用 Action::Goal 类型定义 ActionGoal (Define ActionGoal as type Action::Goal)
  using ActionGoal = Action::Goal;

public:
  /**
   * @brief 构造函数 (Constructor)
   * @param condition_name 条件名称 (Condition name)
   * @param conf 节点配置 (Node configuration)
   */
  WouldAPlannerRecoveryHelp(const std::string & condition_name, const BT::NodeConfiguration & conf);

  // 删除默认构造函数 (Delete the default constructor)
  WouldAPlannerRecoveryHelp() = delete;
};

} // namespace nav2_behavior_tree
#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__WOULD_A_PLANNER_RECOVERY_HELP_CONDITION_HPP_
