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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ARE_ERROR_CODES_PRESENT_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ARE_ERROR_CODES_PRESENT_CONDITION_HPP_

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @class AreErrorCodesPresent
 * @brief 判断给定的错误代码是否存在于需要检查的错误代码集合中 (Check if the given error codes are present in the set of error codes to check)
 */
class AreErrorCodesPresent : public BT::ConditionNode
{
public:
  /**
   * @brief 构造函数 (Constructor)
   * @param condition_name 条件节点名称 (Condition node name)
   * @param conf 节点配置 (Node configuration)
   */
  AreErrorCodesPresent(const std::string & condition_name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf)
  {
    // 获取输入端口 "error_codes_to_check" 的值 (Get the value of the input port "error_codes_to_check")
    getInput<std::set<unsigned short>>("error_codes_to_check", error_codes_to_check_); //NOLINT
  }

  // 删除默认构造函数 (Delete the default constructor)
  AreErrorCodesPresent() = delete;

  /**
   * @brief 执行检查操作，判断当前错误代码是否在需要检查的错误代码集合中 (Perform the check operation to determine if the current error code is in the set of error codes to be checked)
   * @return 返回 BT::NodeStatus::SUCCESS 如果当前错误代码在需要检查的错误代码集合中；否则返回 BT::NodeStatus::FAILURE (Return BT::NodeStatus::SUCCESS if the current error code is in the set of error codes to be checked; otherwise return BT::NodeStatus::FAILURE)
   */
  BT::NodeStatus tick()
  {
    // 获取输入端口 "error_code" 的值 (Get the value of the input port "error_code")
    getInput<unsigned short>("error_code", error_code_); //NOLINT

    // 检查当前错误代码是否在需要检查的错误代码集合中 (Check if the current error code is in the set of error codes to be checked)
    if (error_codes_to_check_.find(error_code_) != error_codes_to_check_.end()) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  /**
   * @brief 提供节点所需的端口列表 (Provide the port list required by the node)
   * @return 返回端口列表 (Return the port list)
   */
  static BT::PortsList providedPorts()
  {
    return {
      // 定义输入端口 "error_code" (Define input port "error_code")
      BT::InputPort<unsigned short>("error_code", "The active error codes"), //NOLINT
      // 定义输入端口 "error_codes_to_check" (Define input port "error_codes_to_check")
      BT::InputPort<std::set<unsigned short>>(
        "error_codes_to_check", "Error codes to check") //NOLINT
    };
  }

protected:
  unsigned short error_code_; ///< 当前错误代码 (Current error code)
  std::set<unsigned short>
    error_codes_to_check_; ///< 需要检查的错误代码集合 (Set of error codes to check)
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ARE_ERROR_CODES_PRESENT_CONDITION_HPP_
