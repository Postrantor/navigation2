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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_TO_POSE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_TO_POSE_ACTION_HPP_

#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief nav2_behavior_tree::BtActionNode 类，包装 nav2_msgs::action::NavigateToPose
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::NavigateToPose
 */
class NavigateToPoseAction : public BtActionNode<nav2_msgs::action::NavigateToPose>
{
  // 使用 Action 类型别名表示 nav2_msgs::action::NavigateToPose
  // Use Action as an alias for nav2_msgs::action::NavigateToPose
  using Action = nav2_msgs::action::NavigateToPose;
  // 使用 ActionResult 类型别名表示 Action::Result
  // Use ActionResult as an alias for Action::Result
  using ActionResult = Action::Result;
  // 使用 ActionGoal 类型别名表示 Action::Goal
  // Use ActionGoal as an alias for Action::Goal

public:
  /**
   * @brief nav2_behavior_tree::NavigateToPoseAction 的构造函数
   * @brief A constructor for nav2_behavior_tree::NavigateToPoseAction
   * @param xml_tag_name XML标签的名称
   * @param action_name 此节点为其创建客户端的操作名称
   * @param conf BT节点配置
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  NavigateToPoseAction(
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
   * @return BT::PortsList 包含基本端口和节点特定端口
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      // 输入端口，目标规划位置
      // Input port for the target planning position
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
      // 输入端口，运行的行为树
      // Input port for the behavior tree to run
      BT::InputPort<std::string>("behavior_tree", "Behavior tree to run"),
      // 输出端口，导航到姿态错误代码
      // Output port for navigate to pose error code
      BT::OutputPort<ActionResult::_error_code_type>(
        "error_code_id", "Navigate to pose error code"),
    });
  }
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_TO_POSE_ACTION_HPP_
