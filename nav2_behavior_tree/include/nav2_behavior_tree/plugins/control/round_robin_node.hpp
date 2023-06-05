// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__ROUND_ROBIN_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__ROUND_ROBIN_NODE_HPP_

#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/control_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief 以循环方式依次执行子节点的序列节点类型 (Type of sequence node that ticks children in a round-robin fashion)
 *
 * 控制节点类型 (Type of Control Node)  | 子节点返回失败 (Child Returns Failure) | 子节点返回运行中 (Child Returns Running)
 * -----------------------------------------------------------------------------------------------
 * 循环 (RoundRobin)                    |      执行下一个子节点 (Tick Next Child)     | 返回运行中 (Return Running)
 *
 * 如果当前子节点返回失败，下一个子节点将被执行，如果最后一个子节点返回失败，
 * 第一个子节点将被执行，循环继续，直到有一个子节点返回成功。
 * (If the current child return failure, the next child is ticked and if the last child returns
 * failure, the first child is ticked and the cycle continues until a child returns success)
 *
 * 例如，假设这个节点有3个子节点：A、B和C。一开始，它们都是IDLE。
 * (As an example, let's say this node has 3 children: A, B and C. At the start,
 * they are all IDLE.)
 * |    A    |    B    |    C    |
 * --------------------------------
 * |  IDLE   |  IDLE   |  IDLE   |
 * | RUNNING |  IDLE   |  IDLE   |  - 首先执行 A，假设它返回 RUNNING
 *                                  - RoundRobin 返回 RUNNING，其他节点不会被执行。
 *                                  (- at first A gets ticked. Assume it returns RUNNING
 *                                  - RoundRobin returns RUNNING and no other nodes are ticked.)
 * | FAILURE | RUNNING |  IDLE   |  - A 返回 FAILURE，所以执行 B 并返回 RUNNING
 *                                  - RoundRobin 返回 RUNNING，C 还没有被执行。
 *                                  (- A returns FAILURE so B gets ticked and returns RUNNING
 *                                  - RoundRobin returns RUNNING and C is not ticked yet)
 * | FAILURE | SUCCESS |  IDLE   |  - B 返回 SUCCESS，所以 RoundRobin 停止所有子节点并
 *                                  - 返回 SUCCESS，下一次迭代将执行 C。
 *                                  (- B returns SUCCESS, so RoundRobin halts all children and
 *                                  - returns SUCCESS, next iteration will tick C.)
 * | RUNNING |  IDLE   | FAILURE |  - C 返回 FAILURE，所以 RoundRobin 循环并执行 A。
 *                                  - A 返回 RUNNING，所以 RoundRobin 返回 RUNNING。
 *                                  (- C returns FAILURE, so RoundRobin circles and ticks A.
 *                                  - A returns RUNNING, so RoundRobin returns RUNNING.)
 *
 * 如果所有子节点都返回 FAILURE，RoundRobin 将返回 FAILURE
 * 并停止所有子节点，结束序列。
 * (If all children return FAILURE, RoundRobin will return FAILURE
 * and halt all children, ending the sequence.)
 *
 * XML 中的用法：<RoundRobin>
 * (Usage in XML: <RoundRobin>)
 */
class RoundRobinNode : public BT::ControlNode
{
public:
  /**
   * @brief nav2_behavior_tree::RoundRobinNode 的构造函数 (A constructor for nav2_behavior_tree::RoundRobinNode)
   * @param name 用于此节点的 XML 标签的名称 (Name for the XML tag for this node)
   */
  explicit RoundRobinNode(const std::string & name);

  /**
   * @brief nav2_behavior_tree::RoundRobinNode 的构造函数 (A constructor for nav2_behavior_tree::RoundRobinNode)
   * @param name 用于此节点的 XML 标签的名称 (Name for the XML tag for this node)
   * @param config BT 节点配置 (BT node configuration)
   */
  RoundRobinNode(const std::string & name, const BT::NodeConfiguration & config);

  /**
   * @brief 主要需要由 BT 动作覆盖的方法 (The main override required by a BT action)
   * @return BT::NodeStatus 执行 tick 的状态 (Status of tick execution)
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 可选的，由 BT 动作重置节点状态所需的其他覆盖方式 (The other (optional) override required by a BT action to reset node state)
   */
  void halt() override;

  /**
   * @brief 创建 BT 端口列表 (Creates list of BT ports)
   * @return BT::PortsList 包含基本端口和节点特定端口的列表 (Containing basic ports along with node-specific ports)
   */
  static BT::PortsList providedPorts() { return {}; }

private:
  unsigned int current_child_idx_{0};   // 当前子节点索引 (Current child index)
  unsigned int num_failed_children_{0}; // 失败子节点数量 (Number of failed children)
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__ROUND_ROBIN_NODE_HPP_
