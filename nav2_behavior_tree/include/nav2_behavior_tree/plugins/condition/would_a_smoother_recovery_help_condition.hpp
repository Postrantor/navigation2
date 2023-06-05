// Copyright (c) 2023 Joshua Wallace
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__WOULD_A_SMOOTHER_RECOVERY_HELP_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__WOULD_A_SMOOTHER_RECOVERY_HELP_CONDITION_HPP_

#include <string>

#include "nav2_behavior_tree/plugins/condition/are_error_codes_present_condition.hpp"
#include "nav2_msgs/action/smooth_path.hpp"

namespace nav2_behavior_tree
{

/**
 * @class WouldASmootherRecoveryHelp
 * @brief 一个继承自 AreErrorCodesPresent 的类，用于检查是否需要平滑路径恢复 (A class derived from AreErrorCodesPresent, used to check if a smoother path recovery is needed)
 */
class WouldASmootherRecoveryHelp : public AreErrorCodesPresent
{
  // 使用 nav2_msgs::action::SmoothPath 定义 Action 类型
  // Define Action type using nav2_msgs::action::SmoothPath
  using Action = nav2_msgs::action::SmoothPath;

  // 使用 Action::Goal 定义 ActionGoal 类型
  // Define ActionGoal type using Action::Goal
  using ActionGoal = Action::Goal;

public:
  /**
   * @brief 构造函数，初始化 WouldASmootherRecoveryHelp 类的实例 (Constructor, initializes an instance of the WouldASmootherRecoveryHelp class)
   *
   * @param condition_name 条件名称 (Condition name)
   * @param conf 节点配置 (Node configuration)
   */
  WouldASmootherRecoveryHelp(
    const std::string & condition_name, const BT::NodeConfiguration & conf);

  /**
   * @brief 删除默认构造函数 (Delete the default constructor)
   */
  WouldASmootherRecoveryHelp() = delete;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__WOULD_A_SMOOTHER_RECOVERY_HELP_CONDITION_HPP_
