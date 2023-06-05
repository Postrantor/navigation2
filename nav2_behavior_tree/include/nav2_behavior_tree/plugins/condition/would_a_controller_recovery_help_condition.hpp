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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__WOULD_A_CONTROLLER_RECOVERY_HELP_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__WOULD_A_CONTROLLER_RECOVERY_HELP_CONDITION_HPP_

#include <string>

#include "nav2_behavior_tree/plugins/condition/are_error_codes_present_condition.hpp"
#include "nav2_msgs/action/follow_path.hpp"

namespace nav2_behavior_tree
{

/**
 * @class WouldAControllerRecoveryHelp
 * @brief 判断控制器恢复是否有助于解决错误 (Determine if a controller recovery would help resolve errors)
 *
 * 这个类继承自 AreErrorCodesPresent，用于判断控制器的恢复动作是否能够帮助解决当前存在的错误。
 * (This class inherits from AreErrorCodesPresent and is used to determine if the recovery action of the controller can help resolve existing errors.)
 */
class WouldAControllerRecoveryHelp : public AreErrorCodesPresent
{
  // 使用 FollowPath 动作 (Use FollowPath action)
  using Action = nav2_msgs::action::FollowPath;
  // 使用 Goal 类型为 FollowPath 动作的目标 (Use Goal type for the target of FollowPath action)
  using ActionGoal = Action::Goal;

public:
  /**
   * @brief 构造函数 (Constructor)
   *
   * @param condition_name 条件名称 (Condition name)
   * @param conf 节点配置 (Node configuration)
   */
  WouldAControllerRecoveryHelp(
    const std::string & condition_name, const BT::NodeConfiguration & conf);

  /**
   * @brief 删除默认构造函数 (Delete default constructor)
   */
  WouldAControllerRecoveryHelp() = delete;
};

} // namespace nav2_behavior_tree
#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__WOULD_A_CONTROLLER_RECOVERY_HELP_CONDITION_HPP_
