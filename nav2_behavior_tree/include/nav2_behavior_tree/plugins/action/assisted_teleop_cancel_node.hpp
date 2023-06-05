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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ASSISTED_TELEOP_CANCEL_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ASSISTED_TELEOP_CANCEL_NODE_HPP_

#include <memory>
#include <string>

#include "nav2_msgs/action/assisted_teleop.hpp"

#include "nav2_behavior_tree/bt_cancel_action_node.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::BackUp
 */
// 这是一个包装 nav2_msgs::action::BackUp 的 nav2_behavior_tree::BtActionNode 类

class AssistedTeleopCancel : public BtCancelActionNode<nav2_msgs::action::AssistedTeleop>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::BackUpAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  // nav2_behavior_tree::BackUpAction 的构造函数
  // 参数 xml_tag_name 是此节点的 XML 标签名
  // 参数 action_name 是此节点创建客户端的动作名称
  // 参数 conf 是 BT 节点配置
  AssistedTeleopCancel(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  // 创建 BT 端口列表
  // 返回值 BT::PortsList 包含基本端口以及节点特定端口
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ASSISTED_TELEOP_CANCEL_NODE_HPP_
