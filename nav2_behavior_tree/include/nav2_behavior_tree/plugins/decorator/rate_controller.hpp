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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief 一个 BT::DecoratorNode，以指定的频率触发其子节点 (A BT::DecoratorNode that ticks its child at a specified rate)
 */
class RateController : public BT::DecoratorNode
{
public:
  /**
   * @brief nav2_behavior_tree::RateController 的构造函数 (A constructor for nav2_behavior_tree::RateController)
   * @param name 此节点的 XML 标签名称 (Name for the XML tag for this node)
   * @param conf BT 节点配置 (BT node configuration)
   */
  RateController(const std::string & name, const BT::NodeConfiguration & conf);

  /**
   * @brief 创建 BT 端口列表 (Creates list of BT ports)
   * @return BT::PortsList 包含特定于节点的端口 (Containing node-specific ports)
   */
  static BT::PortsList providedPorts()
  {
    return {// 输入端口，用于设置频率 (Input port for setting the rate)
            BT::InputPort<double>("hz", 10.0, "Rate")};
  }

private:
  /**
   * @brief 主要覆盖 BT 操作所需的内容 (The main override required by a BT action)
   * @return BT::NodeStatus 执行 tick 的状态 (Status of tick execution)
   */
  BT::NodeStatus tick() override;

  // 记录开始时间点 (Record the start time point)
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  // 周期，单位为秒 (Period in seconds)
  double period_;
  // 标记是否为第一次执行 (Flag for whether it's the first time execution)
  bool first_time_;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_
