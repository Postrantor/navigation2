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

#include "nav2_behavior_tree/plugins/condition/would_a_controller_recovery_help_condition.hpp"
#include <memory>

namespace nav2_behavior_tree
{

/**
 * @brief 构造一个 WouldAControllerRecoveryHelp 对象 (Constructs a WouldAControllerRecoveryHelp object)
 *
 * @param condition_name 条件名称 (Condition name)
 * @param conf 节点配置信息 (Node configuration)
 */
WouldAControllerRecoveryHelp::WouldAControllerRecoveryHelp(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: AreErrorCodesPresent(
    condition_name,
    conf) // 继承自 AreErrorCodesPresent 类，使用 condition_name 和 conf 初始化父类 (Inherits from AreErrorCodesPresent class, initializes the parent class with condition_name and conf)
{
  // 设置 error_codes_to_check_ 成员变量的值为以下错误代码 (Set the value of the error_codes_to_check_ member variable to the following error codes)
  error_codes_to_check_ = {
    ActionGoal::UNKNOWN,                 // 未知错误 (Unknown error)
    ActionGoal::PATIENCE_EXCEEDED,       // 耐心超限错误 (Patience exceeded error)
    ActionGoal::FAILED_TO_MAKE_PROGRESS, // 无法取得进展错误 (Failed to make progress error)
    ActionGoal::NO_VALID_CONTROL         // 无有效控制错误 (No valid control error)
  };
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::WouldAControllerRecoveryHelp>(
    "WouldAControllerRecoveryHelp");
}
