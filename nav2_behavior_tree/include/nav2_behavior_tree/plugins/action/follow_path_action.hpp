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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_

#include <memory>
#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/follow_path.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 包装 nav2_msgs::action::FollowPath 的 nav2_behavior_tree::BtActionNode 类 (A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::FollowPath)
 */
class FollowPathAction : public BtActionNode<nav2_msgs::action::FollowPath>
{
  using Action = nav2_msgs::action::FollowPath;
  using ActionResult = Action::Result;
  using ActionGoal = Action::Goal;

public:
  /**
   * @brief nav2_behavior_tree::FollowPathAction 的构造函数 (A constructor for nav2_behavior_tree::FollowPathAction)
   * @param xml_tag_name 此节点的 XML 标签名 (Name for the XML tag for this node)
   * @param action_name 此节点为其创建客户端的操作名 (Action name this node creates a client for)
   * @param conf BT 节点配置 (BT node configuration)
   */
  FollowPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief 执行用户定义的 tick 操作 (Function to perform some user-defined operation on tick)
   */
  void on_tick() override;

  /**
   * @brief 在操作成功完成后执行用户定义的操作 (Function to perform some user-defined operation upon successful completion of the action)
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief 在操作中止时执行用户定义的操作 (Function to perform some user-defined operation upon abortion of the action)
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief 在操作取消时执行用户定义的操作 (Function to perform some user-defined operation upon cancellation of the action)
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * @brief 在等待尚未收到的结果超时后执行用户定义的操作 (Function to perform some user-defined operation after a timeout waiting for a result that hasn't been received yet)
   * @param feedback 最新反馈消息的 shared_ptr (shared_ptr to latest feedback message)
   */
  void on_wait_for_result(std::shared_ptr<const Action::Feedback> feedback) override;

  /**
   * @brief 创建 BT 端口列表 (Creates list of BT ports)
   * @return 包含基本端口和节点特定端口的 BT::PortsList (BT::PortsList Containing basic ports along with node-specific ports)
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<nav_msgs::msg::Path>("path", "要跟随的路径 (Path to follow)"),
      BT::InputPort<std::string>("controller_id", ""),
      BT::InputPort<std::string>("goal_checker_id", ""),
      BT::OutputPort<ActionResult::_error_code_type>(
        "error_code_id", "跟随路径错误代码 (The follow path error code)"),
    });
  }
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_
