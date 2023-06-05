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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SPIN_CANCEL_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SPIN_CANCEL_NODE_HPP_

#include <memory>
#include <string>

#include "nav2_msgs/action/spin.hpp"

#include "nav2_behavior_tree/bt_cancel_action_node.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief nav2_behavior_tree::BtActionNode 类，封装了 nav2_msgs::action::Wait (A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::Wait)
 */
class SpinCancel : public BtCancelActionNode<nav2_msgs::action::Spin>
{
public:
  /**
   * @brief nav2_behavior_tree::WaitAction 的构造函数 (A constructor for nav2_behavior_tree::WaitAction)
   * @param xml_tag_name 该节点的 XML 标签名 (Name for the XML tag for this node)
   * @param action_name 此节点为其创建客户端的操作名称 (Action name this node creates a client for)
   * @param conf BT 节点配置 (BT node configuration)
   */
  SpinCancel(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief 创建 BT 端口列表 (Creates list of BT ports)
   * @return BT::PortsList 包含基本端口以及特定于节点的端口 (Containing basic ports along with node-specific ports)
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SPIN_CANCEL_NODE_HPP_
