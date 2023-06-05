// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SPEED_CONTROLLER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SPEED_CONTROLLER_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "nav2_util/odometry_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief 一个BT::DecoratorNode，它根据机器人的速度按比例调整子节点的tick频率。
 * 如果机器人行驶得更快，此节点将以更高的频率tick其子节点，并在机器人减速时降低tick频率。
 * (A BT::DecoratorNode that ticks its child every at a rate proportional to
 * the speed of the robot. If the robot travels faster, this node will tick its child at a
 * higher frequency and reduce the tick frequency if the robot slows down)
 */
class SpeedController : public BT::DecoratorNode
{
public:
  /**
   * @brief nav2_behavior_tree::SpeedController的构造函数
   * (A constructor for nav2_behavior_tree::SpeedController)
   * @param name 此节点的XML标签名称 (Name for the XML tag for this node)
   * @param conf BT节点配置 (BT node configuration)
   */
  SpeedController(const std::string & name, const BT::NodeConfiguration & conf);

  /**
   * @brief 创建BT端口列表 (Creates list of BT ports)
   * @return BT::PortsList 包含特定于节点的端口 (Containing node-specific ports)
   */
  static BT::PortsList providedPorts()
  {
    return {
      // 最小速率 (Minimum rate)
      BT::InputPort<double>("min_rate", 0.1, "Minimum rate"),
      // 最大速率 (Maximum rate)
      BT::InputPort<double>("max_rate", 1.0, "Maximum rate"),
      // 最小速度 (Minimum speed)
      BT::InputPort<double>("min_speed", 0.0, "Minimum speed"),
      // 最大速度 (Maximum speed)
      BT::InputPort<double>("max_speed", 0.5, "Maximum speed"),
    };
  }

private:
  /**
   * @brief 主要的 BT action 执行函数 (The main override required by a BT action)
   * @return BT::NodeStatus Tick 执行状态 (Status of tick execution)
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 根据速度限制缩放速率 (Scale the rate based on speed limits)
   * @param speed 当前速度 (Current speed)
   * @return double 缩放后的速率，已进行限幅处理 (Rate scaled by speed limits and clamped)
   */
  inline double getScaledRate(const double & speed)
  {
    // 计算缩放后的速率并进行限幅处理 (Calculate the scaled rate and clamp it)
    return std::max(
      std::min((((speed - min_speed_) / d_speed_) * d_rate_) + min_rate_, max_rate_), min_rate_);
  }

  /**
   * @brief 根据当前平滑速度更新周期，并重置计时器 (Update period based on current smoothed speed and reset timer)
   */
  inline void updatePeriod()
  {
    // 获取平滑后的速度 (Get the smoothed velocity)
    auto velocity = odom_smoother_->getTwist();
    // 计算速度大小 (Calculate the speed magnitude)
    double speed = std::hypot(velocity.linear.x, velocity.linear.y);
    // 计算缩放后的速率 (Calculate the scaled rate)
    double rate = getScaledRate(speed);
    // 更新周期 (Update the period)
    period_ = 1.0 / rate;
  }

  // ROS2 节点 (ROS2 Node)
  rclcpp::Node::SharedPtr node_;

  // 记录开始时间以便重置 (To keep track of time to reset)
  rclcpp::Time start_;

  // 获取平滑速度的对象 (To get a smoothed velocity)
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;

  // 记录是否为第一次执行 tick() 函数 (Whether it's the first call to the tick() function)
  bool first_tick_;

  // 子节点应该被 tick 的时间周期 (Time period after which child node should be ticked)
  double period_;

  // 触发子节点 tick 的速率阈值 (Rates thresholds to tick child node)
  double min_rate_;
  double max_rate_;
  double d_rate_;

  // 速度阈值 (Speed thresholds)
  double min_speed_;
  double max_speed_;
  double d_speed_;

  // 当前目标位置 (Current goal position)
  geometry_msgs::msg::PoseStamped goal_;
  // 目标位置列表 (List of goal positions)
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SPEED_CONTROLLER_HPP_
