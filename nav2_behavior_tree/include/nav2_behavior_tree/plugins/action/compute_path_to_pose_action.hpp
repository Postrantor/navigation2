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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_TO_POSE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_TO_POSE_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.h"

namespace nav2_behavior_tree
{

/**
 * @brief 一个封装了 nav2_msgs::action::ComputePathToPose 的 nav2_behavior_tree::BtActionNode 类
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::ComputePathToPose
 */
class ComputePathToPoseAction : public BtActionNode<nav2_msgs::action::ComputePathToPose>
{
  // 定义 Action、ActionResult 和 ActionGoal 类型别名
  // Define the Action, ActionResult, and ActionGoal type aliases
  using Action = nav2_msgs::action::ComputePathToPose;
  using ActionResult = Action::Result;
  using ActionGoal = Action::Goal;

public:
  /**
   * @brief nav2_behavior_tree::ComputePathToPoseAction 的构造函数
   * @brief A constructor for nav2_behavior_tree::ComputePathToPoseAction
   * @param xml_tag_name 此节点的 XML 标签名
   * @param action_name 此节点创建客户端的操作名
   * @param conf BT 节点配置
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  ComputePathToPoseAction(
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
   * @brief 在操作中止后执行一些用户定义的操作的函数
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief 在操作取消后执行一些用户定义的操作的函数
   * @brief Function to perform some user-defined operation upon cancellation of the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * \brief 由 BT 操作所需的覆盖。取消操作并设置路径输出
   * \brief Override required by a BT action. Cancel the action and set the path output
   */
  void halt() override;

  /**
   * @brief 创建 BT 端口列表
   * @brief Creates list of BT ports
   * @return BT::PortsList 包含基本端口和节点特定端口的列表
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      // 输入端口：目标位置、起始位置、规划器插件类型
      // 输出端口：计算得到的路径、错误代码
      // Input ports: goal position, start position, planner plugin type
      // Output ports: computed path, error code
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "start", "Start pose of the path if overriding current robot pose"),
      BT::InputPort<std::string>("planner_id", "", "Mapped name to the planner plugin type to use"),
      BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputePathToPose node"),
      BT::OutputPort<ActionResult::_error_code_type>(
        "error_code_id", "The compute path to pose error code"),
    });
  }
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_TO_POSE_ACTION_HPP_
