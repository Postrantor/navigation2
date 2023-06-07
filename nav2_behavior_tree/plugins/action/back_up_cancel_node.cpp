// Copyright (c) 2022 Neobotix GmbH
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

#include "nav2_behavior_tree/plugins/action/back_up_cancel_node.hpp"

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

namespace nav2_behavior_tree {

/**
 * @brief BackUpCancel 类的构造函数，继承自 BtCancelActionNode。
 *
 * @param xml_tag_name 该节点在 XML 树中的标签名。
 * @param action_name 执行备份操作的行为名称。
 * @param conf 行为树节点的配置信息。
 */
BackUpCancel::BackUpCancel(
    const std::string& xml_tag_name,
    const std::string& action_name,
    const BT::NodeConfiguration& conf)
    : BtCancelActionNode<nav2_msgs::action::BackUp>(xml_tag_name, action_name, conf) {}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory) {
  /**
   * @brief 注册 BackUpCancel 节点到行为树工厂中。
   *
   * 使用 lambda 表达式创建一个 NodeBuilder 对象，该对象可以用于创建 BackUpCancel 节点。
   *
   * @param name 节点名称。
   * @param config 节点配置信息。
   * @return 返回一个指向 BackUpCancel 节点的智能指针。
   */
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<nav2_behavior_tree::BackUpCancel>(name, "backup", config);
  };

  factory.registerBuilder<nav2_behavior_tree::BackUpCancel>("CancelBackUp", builder);
}
