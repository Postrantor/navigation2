// Copyright (c) 2018 Samsung Research America
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/wait.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 一个包装 nav2_msgs::action::Wait 的 nav2_behavior_tree::BtActionNode 类 (A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::Wait)
 */
class WaitAction : public BtActionNode<nav2_msgs::action::Wait>
{
public:
  /**
   * @brief nav2_behavior_tree::WaitAction 的构造函数 (A constructor for nav2_behavior_tree::WaitAction)
   * @param xml_tag_name 此节点的 XML 标签名称 (Name for the XML tag for this node)
   * @param action_name 此节点为其创建客户端的操作名称 (Action name this node creates a client for)
   * @param conf BT 节点配置 (BT node configuration)
   */
  WaitAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief 在 tick 上执行某些用户定义的操作的功能 (Function to perform some user-defined operation on tick)
   */
  void on_tick() override;

  /**
   * @brief 创建 BT 端口列表 (Creates list of BT ports)
   * @return 包含基本端口和特定于节点的端口的 BT::PortsList (BT::PortsList Containing basic ports along with node-specific ports)
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        // 输入端口，用于设置等待时间（以秒为单位）(Input port for setting wait duration in seconds)
        BT::InputPort<int>("wait_duration", 1, "等待时间 (Wait time)")
      });
  }
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_ACTION_HPP_
