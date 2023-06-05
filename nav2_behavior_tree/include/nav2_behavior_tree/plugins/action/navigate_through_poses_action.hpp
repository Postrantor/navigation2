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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_THROUGH_POSES_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_THROUGH_POSES_ACTION_HPP_

#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 导航行为树节点，封装了nav2_msgs::action::NavigateThroughPoses
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::NavigateThroughPoses
 */
class NavigateThroughPosesAction : public BtActionNode<nav2_msgs::action::NavigateThroughPoses>
{
  using Action = nav2_msgs::action::NavigateThroughPoses;
  using ActionResult = Action::Result;
  using ActionGoal = Action::Goal;

public:
  /**
   * @brief 构造函数，用于创建nav2_behavior_tree::NavigateThroughPosesAction实例
   * @brief A constructor for nav2_behavior_tree::NavigateThroughPosesAction
   * @param xml_tag_name XML标签名
   * @param action_name 该节点创建的动作客户端名称
   * @param conf 行为树节点配置
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  NavigateThroughPosesAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief 在每次tick时执行自定义操作的函数
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief 动作成功完成时执行自定义操作的函数
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief 动作被中止时执行自定义操作的函数
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief 动作被取消时执行自定义操作的函数
   * @brief Function to perform some user-defined operation upon cancellation of the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * @brief 创建行为树端口列表
   * @brief Creates list of BT ports
   * @return BT::PortsList 包含基本端口和节点特定端口的列表
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      // 输入端口，用于设置目标位置
      // Input port for setting destinations to plan through
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goals", "Destinations to plan through"),

      // 输入端口，用于运行行为树
      // Input port for running behavior tree
      BT::InputPort<std::string>("behavior_tree", "Behavior tree to run"),

      // 输出端口，用于返回导航错误代码
      // Output port for returning navigate through poses error code
      BT::OutputPort<ActionResult::_error_code_type>(
        "error_code_id", "The navigate through poses error code"),
    });
  }
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_THROUGH_POSES_ACTION_HPP_
