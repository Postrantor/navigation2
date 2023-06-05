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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PIPELINE_SEQUENCE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PIPELINE_SEQUENCE_HPP_

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/control_node.h"
#include <string>

namespace nav2_behavior_tree
{

/** @brief 重复执行之前的子节点直到返回运行状态的序列节点类型 (Type of sequence node that re-ticks previous children when a child returns running)
 *
 * 控制节点类型 (Type of Control Node)  | 子节点返回失败 (Child Returns Failure) | 子节点返回运行中 (Child Returns Running)
 * -----------------------------------------------------------------------------------------------
 *  PipelineSequence                     |      重新开始 (Restart)               | 再次执行所有先前的节点 (Tick All Previous Again)
 *
 * 再次执行所有先前的节点意味着直到这个节点为止的每个节点都将被重新执行。即使先前的节点返回运行中，下一个节点也会被重新执行。
 * (Tick All Previous Again means every node up till this one will be reticked. Even
 * if a previous node returns Running, the next node will be reticked.)
 *
 * 举个例子，假设这个节点有3个子节点：A、B和C。一开始，它们都是IDLE状态。
 * (As an example, let's say this node has 3 children: A, B and C. At the start,
 * they are all IDLE.)
 * |    A    |    B    |    C    |
 * --------------------------------
 * |  空闲   |  空闲   |  空闲   |  (|  IDLE   |  IDLE   |  IDLE   |)
 * | 运行中  |  空闲   |  空闲   |  - 首先A得到执行。假设它返回运行中
 *                                  - PipelineSequence返回运行中，其他节点不会被执行。
 *                                  (| RUNNING |  IDLE   |  IDLE   |  - at first A gets ticked. Assume it returns RUNNING
 *                                  - PipelineSequence returns RUNNING and no other nodes are ticked.)
 * | 成功    | 运行中  |  空闲   |  - 这次A返回成功，所以B也会被执行
 *                                  - PipelineSequence返回运行中，C还没有被执行
 *                                  (| SUCCESS | RUNNING |  IDLE   |  - This time A returns SUCCESS so B gets ticked as well
 *                                  - PipelineSequence returns RUNNING and C is not ticked yet)
 * | 运行中  | 成功    | 运行中  |  - A得到执行并返回运行中，但由于之前已经返回成功，PipelineSequence继续执行B。
 *                                  - 由于B也返回成功，这次C也被执行。
 *                                  (| RUNNING | SUCCESS | RUNNING |  - A gets ticked and returns RUNNING, but since it had previously
 *                                  - returned SUCCESS, PipelineSequence continues on and ticks B.
 *                                  - Since B also returns SUCCESS, C gets ticked this time as well.)
 * | 运行中  | 成功    | 成功    |  - A仍然是运行中，B再次返回成功。这次C返回成功，结束序列。PipelineSequence返回成功并暂停A。
 *                                  (| RUNNING | SUCCESS | SUCCESS |  - A is still RUNNING, and B returns SUCCESS again. This time C
 *                                  - returned SUCCESS, ending the sequence. PipelineSequence
 *                                  - returns SUCCESS and halts A.)
 *
 * 如果任何子节点在任何时候返回失败。PipelineSequence将返回失败并暂停所有子节点，结束序列。
 * (If any children at any time had returned FAILURE. PipelineSequence would have returned FAILURE
 * and halted all children, ending the sequence.)
 *
 * XML中的用法：<PipelineSequence>
 * (Usage in XML: <PipelineSequence>)
 */
class PipelineSequence : public BT::ControlNode
{
public:
  /**
   * @brief nav2_behavior_tree::PipelineSequence 的构造函数 (A constructor for nav2_behavior_tree::PipelineSequence)
   * @param name 此节点的 XML 标签名 (Name for the XML tag for this node)
   */
  explicit PipelineSequence(const std::string & name);

  /**
   * @brief nav2_behavior_tree::PipelineSequence 的构造函数 (A constructor for nav2_behavior_tree::PipelineSequence)
   * @param name 此节点的 XML 标签名 (Name for the XML tag for this node)
   * @param config BT 节点配置 (BT node configuration)
   */
  PipelineSequence(const std::string & name, const BT::NodeConfiguration & config);

  /**
   * @brief 由 BT 操作所需的其他（可选）覆盖，以重置节点状态 (The other (optional) override required by a BT action to reset node state)
   */
  void halt() override;

  /**
   * @brief 创建 BT 端口列表 (Creates list of BT ports)
   * @return BT::PortsList 包含基本端口和特定于节点的端口 (Containing basic ports along with node-specific ports)
   */
  static BT::PortsList providedPorts() { return {}; }

protected:
  /**
   * @brief 由 BT 操作所需的主要覆盖 (The main override required by a BT action)
   * @return BT::NodeStatus 执行状态 (Status of tick execution)
   */
  BT::NodeStatus tick() override;

  std::size_t last_child_ticked_ = 0;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PIPELINE_SEQUENCE_HPP_
