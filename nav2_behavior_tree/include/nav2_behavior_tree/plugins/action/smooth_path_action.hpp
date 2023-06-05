// Copyright (c) 2021 RoboTech Vision
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SMOOTH_PATH_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SMOOTH_PATH_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav_msgs/msg/path.h"

namespace nav2_behavior_tree
{

/**
 * @brief 一个封装了 nav2_msgs::action::SmoothPath 的 nav2_behavior_tree::BtActionNode 类
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::SmoothPath
 */
class SmoothPathAction : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::SmoothPath>
{
  using Action = nav2_msgs::action::SmoothPath;
  using ActionResult = Action::Result;
  using ActionGoal = Action::Goal;

public:
  /**
   * @brief nav2_behavior_tree::SmoothPathAction 的构造函数
   * @brief A constructor for nav2_behavior_tree::SmoothPathAction
   * @param xml_tag_name 该节点的 XML 标签名
   * @param action_name 该节点创建客户端的操作名
   * @param conf BT 节点配置
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  SmoothPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief 在 tick 上执行一些用户定义的操作的函数
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief 在操作成功完成后执行一些用户定义的操作的函数
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief 在操作中止时执行一些用户定义的操作的函数
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief 在操作取消时执行一些用户定义的操作的函数
   * @brief Function to perform some user-defined operation upon cancellation of the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * @brief 创建 BT 端口列表
   * @brief Creates list of BT ports
   * @return 包含基本端口和节点特定端口的 BT::PortsList
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<nav_msgs::msg::Path>("unsmoothed_path", "需要平滑的路径"),
      BT::InputPort<double>("max_smoothing_duration", 3.0, "最大平滑持续时间"),
      BT::InputPort<bool>("check_for_collisions", false, "如果为 true，则在平滑后执行碰撞检查"),
      BT::InputPort<std::string>("smoother_id", ""),
      BT::OutputPort<nav_msgs::msg::Path>("smoothed_path", "由 SmootherServer 节点平滑的路径"),
      BT::OutputPort<double>("smoothing_duration", "平滑路径所需的时间"),
      BT::OutputPort<bool>("was_completed", "如果平滑未被时间限制中断，则为 True"),
      BT::OutputPort<ActionResult::_error_code_type>("error_code_id", "平滑路径错误代码"),
    });
  }
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SMOOTH_PATH_ACTION_HPP_
