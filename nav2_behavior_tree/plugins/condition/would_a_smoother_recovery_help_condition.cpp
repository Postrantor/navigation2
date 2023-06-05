// Copyright (c) 2023 Joshua Wallace
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

#include "nav2_behavior_tree/plugins/condition/would_a_smoother_recovery_help_condition.hpp"
#include <memory>

namespace nav2_behavior_tree
{

// 这是一个类构造函数，用于初始化 WouldASmootherRecoveryHelp 类的实例。它接受两个参数：condition_name 和 conf。condition_name 是一个字符串，表示条件名称；conf 是节点配置。在构造函数中，首先调用基类 AreErrorCodesPresent 的构造函数，并传入相同的参数。然后，为成员变量 error_codes_to_check_ 分配一个包含四个错误代码的列表：UNKNOWN、TIMEOUT、FAILED_TO_SMOOTH_PATH 和 SMOOTHED_PATH_IN_COLLISION。这些错误代码将在后续操作中进行检查，以确定是否需要更平滑的恢复路径。
/**
 * @brief 用于判断是否需要更平滑的恢复路径的类构造函数 (Constructor for the class to determine if a smoother recovery path is needed)
 * 
 * @param condition_name 条件名称 (Condition name)
 * @param conf 节点配置 (Node configuration)
 */
WouldASmootherRecoveryHelp::WouldASmootherRecoveryHelp(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: AreErrorCodesPresent(condition_name, conf) // 调用基类构造函数 (Calling base class constructor)
{
  // 需要检查的错误代码列表 (List of error codes to check)
  error_codes_to_check_ = {
    ActionGoal::UNKNOWN,                     // 未知错误 (Unknown error)
    ActionGoal::TIMEOUT,                     // 超时错误 (Timeout error)
    ActionGoal::FAILED_TO_SMOOTH_PATH,       // 平滑路径失败 (Failed to smooth path)
    ActionGoal::SMOOTHED_PATH_IN_COLLISION}; // 平滑后路径发生碰撞 (Smoothed path in collision)
}

} // namespace nav2_behavior_tree

// 导入 BehaviorTree 库中的 bt_factory.h 头文件
// Include the bt_factory.h header file from the BehaviorTree library
#include "behaviortree_cpp_v3/bt_factory.h"

/** 
 * @fn BT_REGISTER_NODES
 * @brief 注册 BehaviorTree 节点，使其可以在树中使用
 * @brief Register BehaviorTree nodes so that they can be used within the tree
 *
 * @param factory BehaviorTree 的工厂对象，用于注册节点类型
 * @param factory Factory object of BehaviorTree, used for registering node types
 */
BT_REGISTER_NODES(factory)
{
  // 使用 factory 对象注册自定义节点类型 nav2_behavior_tree::WouldASmootherRecoveryHelp
  // 并为其分配一个字符串名称 "WouldASmootherRecoveryHelp"
  // Register the custom node type nav2_behavior_tree::WouldASmootherRecoveryHelp with the factory object
  // and assign it a string name "WouldASmootherRecoveryHelp"
  factory.registerNodeType<nav2_behavior_tree::WouldASmootherRecoveryHelp>(
    "WouldASmootherRecoveryHelp");
}
