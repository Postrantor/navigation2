// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_

#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to shorten path by some distance
 * @brief 一个用于缩短路径一定距离的 BT::ActionNodeBase
 */
class TruncatePath : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::TruncatePath constructor
   * @brief nav2_behavior_tree::TruncatePath 的构造函数
   * @param xml_tag_name Name for the XML tag for this node
   * @param xml_tag_name 此节点的 XML 标签名称
   * @param conf BT node configuration
   * @param conf BT 节点配置
   */
  TruncatePath(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @brief 创建 BT 端口列表
   * @return BT::PortsList Containing basic ports along with node-specific ports
   * @return BT::PortsList 包含基本端口以及特定于节点的端口
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Original Path"),
      // 输入端口，接收原始路径
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Path truncated to a certain distance"),
      // 输出端口，输出被截断到一定距离的路径
      BT::InputPort<double>("distance", 1.0, "distance"),
      // 输入端口，接收要截断的距离
    };
  }

private:
  /**
   * @brief The other (optional) override required by a BT action.
   * @brief 另一个（可选的）BT 操作所需的重写。
   */
  void halt() override {}

  /**
   * @brief The main override required by a BT action
   * @brief BT 操作所需的主要重写
   * @return BT::NodeStatus Status of tick execution
   * @return BT::NodeStatus 执行 tick 的状态
   */
  BT::NodeStatus tick() override;

  double distance_;
  // 要截断的距离
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_
