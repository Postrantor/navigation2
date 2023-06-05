// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TIME_EXPIRED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TIME_EXPIRED_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 一个 BT::ConditionNode，在指定的时间周期过去时返回 SUCCESS，否则返回 FAILURE
 *        (A BT::ConditionNode that returns SUCCESS every time a specified
 *        time period passes and FAILURE otherwise)
 */
class TimeExpiredCondition : public BT::ConditionNode
{
public:
  /**
   * @brief nav2_behavior_tree::TimeExpiredCondition 的构造函数
   *        (A constructor for nav2_behavior_tree::TimeExpiredCondition)
   * @param condition_name 此节点的 XML 标签名称 (Name for the XML tag for this node)
   * @param conf BT 节点配置 (BT node configuration)
   */
  TimeExpiredCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

  // 删除默认构造函数 (Delete the default constructor)
  TimeExpiredCondition() = delete;

  /**
   * @brief 主要需要覆盖的 BT 动作 (The main override required by a BT action)
   * @return BT::NodeStatus 执行 tick 的状态 (Status of tick execution)
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 创建 BT 端口列表 (Creates list of BT ports)
   * @return BT::PortsList 包含特定于节点的端口 (Containing node-specific ports)
   */
  static BT::PortsList providedPorts()
  {
    return {// 输入端口，包含秒数 (Input port with seconds)
            BT::InputPort<double>("seconds", 1.0, "Seconds")};
  }

private:
  rclcpp::Node::SharedPtr node_; // 共享指针类型的节点 (Shared pointer type node)
  rclcpp::Time start_;           // 开始时间 (Start time)
  double period_;                // 时间周期 (Time period)
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TIME_EXPIRED_CONDITION_HPP_
