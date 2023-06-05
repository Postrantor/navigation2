// Copyright (c) 2021 Samsung Research America
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SINGLE_TRIGGER_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SINGLE_TRIGGER_NODE_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief 一个BT::DecoratorNode，仅触发一次子节点并在每个后续tick返回FAILURE。
 *        A BT::DecoratorNode that triggers its child only once and returns FAILURE for every succeeding tick.
 */
class SingleTrigger : public BT::DecoratorNode
{
public:
  /**
   * @brief nav2_behavior_tree::SingleTrigger的构造函数。
   *        A constructor for nav2_behavior_tree::SingleTrigger.
   * @param name 此节点的XML标签名称。
   *             Name for the XML tag for this node.
   * @param conf BT节点配置。
   *             BT node configuration.
   */
  SingleTrigger(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief 创建BT端口列表。
   *        Creates list of BT ports.
   * @return BT::PortsList 包含节点特定端口的列表。
   *         BT::PortsList Containing node-specific ports.
   */
  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  /**
   * @brief 由BT操作需要的主要覆盖。
   *        The main override required by a BT action.
   * @return BT::NodeStatus tick执行状态。
   *         BT::NodeStatus Status of tick execution.
   */
  BT::NodeStatus tick() override;

  bool first_time_; // 初始时为true，表示第一次触发。当触发子节点后，将其设为false。Initial value is true, representing the first trigger. Set to false after triggering the child node.
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SINGLE_TRIGGER_NODE_HPP_
