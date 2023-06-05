// Copyright (c) 2022 Neobotix GmbH
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_LONGER_ON_APPROACH_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_LONGER_ON_APPROACH_HPP_

#include <limits>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 一个BT::DecoratorNode，当新路径的长度比旧路径短时，每次都会对其子节点进行tick操作。
 *        (A BT::DecoratorNode that ticks its child every time when the length of
 *        the new path is smaller than the old one by the length given by the user.)
 */
class PathLongerOnApproach : public BT::DecoratorNode
{
public:
  /**
   * @brief nav2_behavior_tree::PathLongerOnApproach 的构造函数
   *        (A constructor for nav2_behavior_tree::PathLongerOnApproach)
   * @param name XML标签中此节点的名称 (Name for the XML tag for this node)
   * @param conf BT节点配置 (BT node configuration)
   */
  PathLongerOnApproach(const std::string & name, const BT::NodeConfiguration & conf);

  /**
   * @brief 创建BT端口列表 (Creates list of BT ports)
   * @return 包含特定于节点的端口的BT::PortsList (BT::PortsList Containing node-specific ports)
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "计划路径 (Planned Path)"),
      BT::InputPort<double>(
        "prox_len", 3.0, "路径在接近时变长的临界距离（米）(Proximity length (m) for the path to be longer on approach)"),
      BT::InputPort<double>(
        "length_factor", 2.0,
        "检查路径是否明显更长的长度乘法因子 (Length multiplication factor to check if the path is significantly longer)"),
    };
  }

  /**
   * @brief 主要覆盖，由BT操作所需 (The main override required by a BT action)
   * @return tick执行的BT::NodeStatus状态 (BT::NodeStatus Status of tick execution)
   */
  BT::NodeStatus tick() override;

private:
  /**
   * @brief 检查全局路径是否更新 (Checks if the global path is updated)
   * @param new_path 新路径到目标 (new path to the goal)
   * @param old_path 当前路径到目标 (current path to the goal)
   * @return 当前目标的路径是否更新 (whether the path is updated for the current goal)
   */
  bool isPathUpdated(nav_msgs::msg::Path & new_path, nav_msgs::msg::Path & old_path);

  /**
   * @brief 检查机器人是否在目标附近 (Checks if the robot is in the goal proximity)
   * @param old_path 当前路径到目标 (current path to the goal)
   * @param prox_leng 距离目标的临近距离 (proximity length from the goal)
   * @return 机器人是否在目标附近 (whether the robot is in the goal proximity)
   */
  bool isRobotInGoalProximity(nav_msgs::msg::Path & old_path, double & prox_leng);

  /**
   * @brief 检查新路径是否更长 (Checks if the new path is longer)
   * @param new_path 新路径到目标 (new path to the goal)
   * @param old_path 当前路径到目标 (current path to the goal)
   * @param length_factor 路径长度检查的乘数 (multiplier for path length check)
   * @return 新路径是否更长 (whether the new path is longer)
   */
  bool isNewPathLonger(
    nav_msgs::msg::Path & new_path, nav_msgs::msg::Path & old_path, double & length_factor);

private:
  nav_msgs::msg::Path new_path_; // 新路径 (new path)
  nav_msgs::msg::Path old_path_; // 旧路径 (old path)
  double prox_len_ = std::numeric_limits<double>::max(); // 临近距离 (proximity length)
  double length_factor_ = std::numeric_limits<double>::max(); // 长度因子 (length factor)
  rclcpp::Node::SharedPtr node_; // 共享指针节点 (shared pointer node)
  bool first_time_ = true; // 首次标志 (first time flag)
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_LONGER_ON_APPROACH_HPP_
