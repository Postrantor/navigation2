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

#include "nav2_behavior_tree/plugins/action/assisted_teleop_cancel_node.hpp"

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

namespace nav2_behavior_tree {

AssistedTeleopCancel::AssistedTeleopCancel(
    const std::string& xml_tag_name,    // XML标签名称
    const std::string& action_name,     // 行为名称
    const BT::NodeConfiguration& conf)  // 节点配置
    : BtCancelActionNode<nav2_msgs::action::AssistedTeleop>(xml_tag_name, action_name, conf) {
}  // 继承BtCancelActionNode类，传入参数

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder =
      [](const std::string& name,
         const BT::NodeConfiguration& config) {  // 定义一个lambda表达式，返回一个unique_ptr指针
        return std::make_unique<
            nav2_behavior_tree::AssistedTeleopCancel>(  // 创建unique_ptr指针，传入参数
            name, "assisted_teleop", config);
      };

  factory
      .registerBuilder<nav2_behavior_tree::AssistedTeleopCancel>(  // 注册AssistedTeleopCancel节点
          "CancelAssistedTeleop", builder);                        // 节点名称和builder函数
}
