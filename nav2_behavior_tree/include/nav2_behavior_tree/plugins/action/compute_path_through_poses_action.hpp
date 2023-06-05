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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_THROUGH_POSES_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_THROUGH_POSES_ACTION_HPP_

#include <string>
#include <vector>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav_msgs/msg/path.h"

namespace nav2_behavior_tree
{

/**
 * @brief 一个封装了 nav2_msgs::action::ComputePathThroughPoses 的 nav2_behavior_tree::BtActionNode 类
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::ComputePathThroughPoses
 */
class ComputePathThroughPosesAction
: public BtActionNode<nav2_msgs::action::ComputePathThroughPoses>
{
  // 定义 Action、ActionResult 和 ActionGoal 类型别名
  // Define Action, ActionResult and ActionGoal type aliases
  using Action = nav2_msgs::action::ComputePathThroughPoses;
  using ActionResult = Action::Result;
  using ActionGoal = Action::Goal;

public:
  /**
   * @brief nav2_behavior_tree::ComputePathThroughPosesAction 的构造函数
   * @brief A constructor for nav2_behavior_tree::ComputePathThroughPosesAction
   * @param xml_tag_name 此节点的 XML 标签名
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name 此节点为其创建客户端的操作名称
   * @param action_name Action name this node creates a client for
   * @param conf BT 节点配置
   * @param conf BT node configuration
   */
  ComputePathThroughPosesAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief 在 tick 上执行一些用户定义的操作的功能
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief 在操作成功完成后执行一些用户定义的操作的功能
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief 在操作中止时执行一些用户定义的操作的功能
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief 在操作取消时执行一些用户定义的操作的功能
   * @brief Function to perform some user-defined operation upon cancelation of the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * @brief 创建 BT 端口列表
   * @brief Creates list of BT ports
   * @return BT::PortsList 包含基本端口及节点特定端口
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      // 输入端口 "goals"，用于规划目标点
      // Input port "goals" for planning target points
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "goals", "Destinations to plan through"),
      // 输入端口 "start"，用于覆盖当前机器人姿态的路径起点
      // Input port "start" for the start pose of the path if overriding current robot pose
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "start", "Start pose of the path if overriding current robot pose"),
      // 输入端口 "planner_id"，映射到要使用的规划插件类型的名称
      // Input port "planner_id" for the mapped name to the planner plugin type to use
      BT::InputPort<std::string>("planner_id", "", "Mapped name to the planner plugin type to use"),
      // 输出端口 "path"，由 ComputePathThroughPoses 节点创建的路径
      // Output port "path" for the path created by ComputePathThroughPoses node
      BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputePathThroughPoses node"),
      // 输出端口 "error_code_id"，计算路径错误代码
      // Output port "error_code_id" for the compute path through poses error code
      BT::OutputPort<ActionResult::_error_code_type>(
        "error_code_id", "The compute path through poses error code"),
    });
  }
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_THROUGH_POSES_ACTION_HPP_
