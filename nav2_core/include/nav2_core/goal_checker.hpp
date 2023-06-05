/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NAV2_CORE__GOAL_CHECKER_HPP_
#define NAV2_CORE__GOAL_CHECKER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_core {

/**
 * @class GoalChecker
 * @brief 用于检查目标是否已达到的函数对象 (Function-object for checking whether a goal has been
 * reached)
 *
 * 此类定义了确定是否达到目标状态的插件接口。这主要包括检查两个姿态的相对位置（假定它们处于同一帧中）。
 * 它还可以检查速度，因为某些应用程序需要机器人停止才能被认为达到了目标。
 * (This class defines the plugin interface for determining whether you have reached
 * the goal state. This primarily consists of checking the relative positions of two poses
 * (which are presumed to be in the same frame). It can also check the velocity, as some
 * applications require that robot be stopped to be considered as having reached the goal.)
 */
class GoalChecker {
public:
  typedef std::shared_ptr<nav2_core::GoalChecker> Ptr;

  // 析构函数 (Destructor)
  virtual ~GoalChecker() {}

  /**
   * @brief 从NodeHandle初始化任何参数 (Initialize any parameters from the NodeHandle)
   * @param parent 用于获取参数的节点指针 (Node pointer for grabbing parameters)
   * @param plugin_name 插件名称 (Plugin name)
   * @param costmap_ros 成本地图的共享指针 (Shared pointer of the costmap)
   */
  virtual void initialize(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
      const std::string& plugin_name,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  // 重置函数 (Reset function)
  virtual void reset() = 0;

  /**
   * @brief 检查目标是否应该被认为已达到 (Check whether the goal should be considered reached)
   * @param query_pose 要检查的姿态 (The pose to check)
   * @param goal_pose 要与之对比的姿态 (The pose to check against)
   * @param velocity 当前机器人的速度 (The robot's current velocity)
   * @return 如果目标已达到，则返回True (True if goal is reached)
   */
  virtual bool isGoalReached(
      const geometry_msgs::msg::Pose& query_pose,
      const geometry_msgs::msg::Pose& goal_pose,
      const geometry_msgs::msg::Twist& velocity) = 0;

  /**
   * @brief 获取用于主要类型目标检查的最大可能公差。 (Get the maximum possible tolerances used for
   * goal checking in the major types.)
   * 没有有效条目的任何字段都会替换为std::numeric_limits<double>::lowest()
   * 表示不进行测量。对于多个条目的公差（例如XY公差），两个字段都将包含此值，
   * 因为它是每个独立字段可能具有的最大公差，假设其他字段没有错误（例如X和Y）。
   * (Any field without a valid entry is replaced with std::numeric_limits<double>::lowest()
   * to indicate that it is not measured. For tolerance across multiple entries
   * (e.x. XY tolerances), both fields will contain this value since it is the maximum tolerance
   * that each independent field could be assuming the other has no error (e.x. X and Y).)
   * @param pose_tolerance 用于检查姿态字段的公差 (The tolerance used for checking in Pose fields)
   * @param vel_tolerance 用于检查速度字段的公差 (The tolerance used for checking velocity fields)
   * @return 如果公差可用，则返回True (True if the tolerances are valid to use)
   */
  virtual bool getTolerances(
      geometry_msgs::msg::Pose& pose_tolerance, geometry_msgs::msg::Twist& vel_tolerance) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__GOAL_CHECKER_HPP_
