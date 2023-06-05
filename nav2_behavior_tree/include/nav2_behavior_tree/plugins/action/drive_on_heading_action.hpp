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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DRIVE_ON_HEADING_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DRIVE_ON_HEADING_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief DriveOnHeadingAction 类，封装了 nav2_msgs::action::DriveOnHeading
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::DriveOnHeading
 */
class DriveOnHeadingAction : public BtActionNode<nav2_msgs::action::DriveOnHeading>
{
public:
  /**
   * @brief nav2_behavior_tree::DriveOnHeadingAction 的构造函数
   * @brief A constructor for nav2_behavior_tree::DriveOnHeadingAction
   * @param xml_tag_name 用于此节点的 XML 标签名称
   * @param action_name 此节点为其创建客户端的操作名称
   * @param conf BT 节点配置
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  DriveOnHeadingAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief 创建 BT 端口列表
   * @brief Creates list of BT ports
   * @return BT::PortsList 包含基本端口以及特定于节点的端口
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<double>("dist_to_travel", 0.15, "要行驶的距离"),
       BT::InputPort<double>("speed", 0.025, "行驶速度"),
       BT::InputPort<double>("time_allowance", 10.0, "允许在航向上行驶的时间")});
  }
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DRIVE_ON_HEADING_ACTION_HPP_
