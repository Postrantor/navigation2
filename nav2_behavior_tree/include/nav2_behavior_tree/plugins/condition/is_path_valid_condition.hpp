// Copyright (c) 2021 Joshua Wallace
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PATH_VALID_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PATH_VALID_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 一个BT::ConditionNode，当IsPathValid服务返回true时返回SUCCESS，否则返回FAILURE
 * @brief A BT::ConditionNode that returns SUCCESS when the IsPathValid
 * service returns true and FAILURE otherwise
 */
class IsPathValidCondition : public BT::ConditionNode
{
public:
  /**
   * @brief nav2_behavior_tree::IsPathValidCondition的构造函数
   * @brief A constructor for nav2_behavior_tree::IsPathValidCondition
   * @param condition_name 此节点的XML标签名称
   * @param condition_name Name for the XML tag for this node
   * @param conf BT节点配置
   * @param conf BT node configuration
   */
  IsPathValidCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

  // 删除默认构造函数
  // Delete the default constructor
  IsPathValidCondition() = delete;

  /**
   * @brief 由BT行为需要的主要覆盖
   * @brief The main override required by a BT action
   * @return BT::NodeStatus 执行tick的状态
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 创建BT端口列表
   * @brief Creates list of BT ports
   * @return BT::PortsList 包含特定于节点的端口
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {// 输入端口，检查路径
            // Input port for the path to check
            BT::InputPort<nav_msgs::msg::Path>("path", "Path to Check"),
            // 输入端口，服务器超时
            // Input port for the server timeout
            BT::InputPort<std::chrono::milliseconds>("server_timeout")};
  }

private:
  // ROS2节点共享指针
  // Shared pointer to a ROS2 node
  rclcpp::Node::SharedPtr node_;
  // IsPathValid服务的客户端共享指针
  // Shared pointer to a client for the IsPathValid service
  rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr client_;
  // 等待is path valid服务响应的超时值
  // The timeout value while waiting for a responce from the
  // is path valid service
  std::chrono::milliseconds server_timeout_;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PATH_VALID_CONDITION_HPP_
