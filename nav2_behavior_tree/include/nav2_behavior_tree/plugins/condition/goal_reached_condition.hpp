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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_REACHED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_REACHED_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

/**
 * @brief 一个 BT::ConditionNode，当达到指定目标时返回 SUCCESS，否则返回 FAILURE
 * @brief A BT::ConditionNode that returns SUCCESS when a specified goal
 * is reached and FAILURE otherwise
 */
class GoalReachedCondition : public BT::ConditionNode
{
public:
  /**
   * @brief nav2_behavior_tree::GoalReachedCondition 的构造函数
   * @brief A constructor for nav2_behavior_tree::GoalReachedCondition
   * @param condition_name 此节点的 XML 标签名
   * @param condition_name Name for the XML tag for this node
   * @param conf BT 节点配置
   * @param conf BT node configuration
   */
  GoalReachedCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

  // 删除默认构造函数
  // Delete the default constructor
  GoalReachedCondition() = delete;

  /**
   * @brief nav2_behavior_tree::GoalReachedCondition 的析构函数
   * @brief A destructor for nav2_behavior_tree::GoalReachedCondition
   */
  ~GoalReachedCondition() override;

  /**
   * @brief 主要的 BT 动作覆盖
   * @brief The main override required by a BT action
   * @return BT::NodeStatus 执行 tick 的状态
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 读取参数并初始化类变量的函数
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief 检查当前机器人姿态是否在距离目标给定距离内
   * @brief Checks if the current robot pose lies within a given distance from the goal
   * @return bool 当目标达到时为 true，否则为 false
   * @return bool true when goal is reached, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief 创建 BT 端口列表
   * @brief Creates list of BT ports
   * @return BT::PortsList 包含特定于节点的端口
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame")};
  }

protected:
  /**
   * @brief 清理函数
   * @brief Cleanup function
   */
  void cleanup() {}

private:
  // ROS2 节点共享指针
  // Shared pointer for the ROS2 node
  rclcpp::Node::SharedPtr node_;

  // tf2_ros::Buffer 的共享指针
  // Shared pointer for the tf2_ros::Buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // 初始化标志
  // Initialization flag
  bool initialized_;

  // 目标达到的容差
  // Tolerance for goal reached
  double goal_reached_tol_;

  // 全局坐标系名称
  // Global frame name
  std::string global_frame_;

  // 机器人基座坐标系名称
  // Robot base frame name
  std::string robot_base_frame_;

  // 坐标变换容差
  // Tolerance for coordinate transformation
  double transform_tolerance_;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_REACHED_CONDITION_HPP_
