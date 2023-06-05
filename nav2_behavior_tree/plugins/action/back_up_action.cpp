// Copyright (c) 2018 Intel Corporation
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

#include "nav2_behavior_tree/plugins/action/back_up_action.hpp"

#include <memory>
#include <string>

/**
 * @brief BackUpAction 构造函数
 *
 * @param xml_tag_name 行为树 XML 标签名称
 * @param action_name 行为名称
 * @param conf 行为节点配置
 */
BackUpAction::BackUpAction(
    const std::string& xml_tag_name,
    const std::string& action_name,
    const BT::NodeConfiguration& conf)
    : BtActionNode<nav2_msgs::action::BackUp>(xml_tag_name, action_name, conf) {
  double dist;
  getInput("backup_dist", dist);               // 获取 backup_dist 输入参数
  double speed;
  getInput("backup_speed", speed);             // 获取 backup_speed 输入参数
  double time_allowance;
  getInput("time_allowance", time_allowance);  // 获取 time_allowance 输入参数

  // 填充输入消息
  goal_.target.x = dist;  // 设置目标距离
  goal_.target.y = 0.0;
  goal_.target.z = 0.0;
  goal_.speed = speed;                                                    // 设置倒车速度
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);  // 设置时间限制
}

/**
 * @brief BackUpAction 的 on_tick 函数
 *
 */
void BackUpAction::on_tick() { increment_recovery_count(); }

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<nav2_behavior_tree::BackUpAction>(name, "backup", config);
  };

  factory.registerBuilder<nav2_behavior_tree::BackUpAction>("BackUp", builder);
}
