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

#include "nav2_behavior_tree/plugins/action/assisted_teleop_action.hpp"

#include <memory>
#include <string>

namespace nav2_behavior_tree {

AssistedTeleopAction::AssistedTeleopAction(
    const std::string& xml_tag_name,           // XML 标签名称
    const std::string& action_name,            // 行为名称
    const BT::NodeConfiguration& conf)         // 行为树节点配置
    : BtActionNode<nav2_msgs::action::AssistedTeleop>(
          xml_tag_name, action_name, conf) {   // 继承父类构造函数
  double time_allowance;                       // 时间限制
  getInput("time_allowance", time_allowance);  // 获取输入参数 "time_allowance" 的值
  getInput("is_recovery", is_recovery_);       // 获取输入参数 "is_recovery" 的值

  // 填充输入消息
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

void AssistedTeleopAction::on_tick() {
  if (is_recovery_) {            // 如果需要恢复
    increment_recovery_count();  // 增加恢复计数器
  }
}

BT::NodeStatus AssistedTeleopAction::on_aborted() {
  return is_recovery_ ? BT::NodeStatus::FAILURE
                      : BT::NodeStatus::SUCCESS;  // 如果需要恢复，返回失败；否则返回成功
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {  // 注册行为树节点
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<nav2_behavior_tree::AssistedTeleopAction>(
        name, "assisted_teleop", config);  // 创建并返回行为树节点
  };

  factory.registerBuilder<nav2_behavior_tree::AssistedTeleopAction>(
      "AssistedTeleop", builder);  // 注册行为树节点构造器
}
