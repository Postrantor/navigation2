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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATER_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATER_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp_v3/decorator_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 一个BT::DecoratorNode，订阅目标主题并更新黑板上的当前目标
 *        A BT::DecoratorNode that subscribes to a goal topic and updates
 *        the current goal on the blackboard
 */
class GoalUpdater : public BT::DecoratorNode
{
public:
  /**
   * @brief nav2_behavior_tree::GoalUpdater 的构造函数
   *        A constructor for nav2_behavior_tree::GoalUpdater
   * @param xml_tag_name XML标签名
   *        Name for the XML tag for this node
   * @param conf BT节点配置
   *        BT node configuration
   */
  GoalUpdater(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  /**
   * @brief 创建BT端口列表
   *        Creates list of BT ports
   * @return 包含特定节点端口的 BT::PortsList
   *         BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal", "原始目标 Original Goal"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "output_goal", "通过订阅接收到的目标 Received Goal by subscription"),
    };
  }

private:
  /**
   * @brief 主要的 BT action 执行覆盖
   *        The main override required by a BT action
   * @return tick执行状态
   *         BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 目标更新主题的回调函数
   *        Callback function for goal update topic
   * @param msg geometry_msgs::msg::PoseStamped 消息的共享指针
   *        Shared pointer to geometry_msgs::msg::PoseStamped message
   */
  void callback_updated_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    goal_sub_; // 目标订阅 Goal subscription

  geometry_msgs::msg::PoseStamped last_goal_received_; // 最后接收到的目标 Last goal received

  rclcpp::Node::SharedPtr node_;                    // 节点共享指针 Node shared pointer
  rclcpp::CallbackGroup::SharedPtr callback_group_; // 回调组共享指针 Callback group shared pointer
  rclcpp::executors::SingleThreadedExecutor
    callback_group_executor_; // 回调组执行器 Callback group executor
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATER_NODE_HPP_
