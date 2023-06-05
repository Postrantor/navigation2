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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_HPP_

#include "behaviortree_cpp_v3/control_node.h"
#include <string>

namespace nav2_behavior_tree
{
/**
 * @brief 复苏节点只有两个子节点，当且仅当第一个子节点返回成功时，才返回成功。
 *
 * - 如果第一个子节点返回失败，则执行第二个子节点。如果第二个子节点返回成功，则再次执行第一个子节点。
 *
 * - 如果第一个或第二个子节点返回正在运行，则此节点返回正在运行。
 *
 * - 如果第二个子节点返回失败，则此控制节点将停止循环并返回失败。
 *
 * @brief The RecoveryNode has only two children and returns SUCCESS if and only if the first child
 * returns SUCCESS.
 *
 * - If the first child returns FAILURE, the second child will be executed.  After that the first
 * child is executed again if the second child returns SUCCESS.
 *
 * - If the first or second child returns RUNNING, this node returns RUNNING.
 *
 * - If the second child returns FAILURE, this control node will stop the loop and returns FAILURE.
 *
 */
class RecoveryNode : public BT::ControlNode
{
public:
  /**
   * @brief nav2_behavior_tree::RecoveryNode 的构造函数
   * @param name 此节点的 XML 标签名称
   * @param conf BT 节点配置
   *
   * @brief A constructor for nav2_behavior_tree::RecoveryNode
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  RecoveryNode(const std::string & name, const BT::NodeConfiguration & conf);

  /**
   * @brief nav2_behavior_tree::RecoveryNode 的析构函数
   *
   * @brief A destructor for nav2_behavior_tree::RecoveryNode
   */
  ~RecoveryNode() override = default;

  /**
   * @brief 创建 BT 端口列表
   * @return BT::PortsList 包含基本端口和节点特定端口的列表
   *
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("number_of_retries", 1, "Number of retries")};
  }

private:
  unsigned int current_child_idx_; // 当前子节点索引 (Current child index)
  unsigned int number_of_retries_; // 重试次数 (Number of retries)
  unsigned int retry_count_;       // 重试计数 (Retry count)

  /**
   * @brief 主要覆盖，由 BT 操作所需
   * @return BT::NodeStatus 执行 tick 的状态
   *
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 另一个（可选）覆盖，由 BT 操作重置节点状态所需
   *
   * @brief The other (optional) override required by a BT action to reset node state
   */
  void halt() override;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_HPP_
