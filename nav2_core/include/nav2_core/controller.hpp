/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  Copyright (c) 2019, Intel Corporation
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

#ifndef NAV2_CORE__CONTROLLER_HPP_
#define NAV2_CORE__CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.h"

namespace nav2_core {

/**
 * @class Controller
 * @brief 控制器接口，作为所有控制器插件的虚拟基类 (controller interface that acts as a virtual base
 * class for all controller plugins)
 */
class Controller {
public:
  using Ptr = std::shared_ptr<nav2_core::Controller>;

  /**
   * @brief 虚拟析构函数 (Virtual destructor)
   */
  virtual ~Controller() {}

  /**
   * @param  parent 指向用户节点的指针 (pointer to user's node)
   * @param  costmap_ros 指向costmap的指针 (A pointer to the costmap)
   */
  virtual void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
      std::string name,
      std::shared_ptr<tf2_ros::Buffer>,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) = 0;

  /**
   * @brief 清理资源的方法 (Method to cleanup resources)
   */
  virtual void cleanup() = 0;

  /**
   * @brief 激活规划器和执行过程中涉及的任何线程的方法 (Method to active planner and any threads
   * involved in execution)
   */
  virtual void activate() = 0;

  /**
   * @brief 停用规划器和执行过程中涉及的任何线程的方法 (Method to deactivate planner and any threads
   * involved in execution)
   */
  virtual void deactivate() = 0;

  /**
   * @brief local setPlan - 设置全局计划 (Sets the global plan)
   * @param path 全局计划 (The global plan)
   */
  virtual void setPlan(const nav_msgs::msg::Path &path) = 0;

  /**
   * @brief 控制器计算速度命令 - 根据当前姿态和速度计算最佳命令 (Controller computeVelocityCommands
   * - calculates the best command given the current pose and velocity)
   *
   * 假定全局计划已经设置 (It is presumed that the global plan is already set)
   *
   * 这主要是对受保护的computeVelocityCommands函数的封装，它具有额外的调试信息 (This is mostly a
   * wrapper for the protected computeVelocityCommands function which has additional debugging info)
   *
   * @param pose 当前机器人姿态 (Current robot pose)
   * @param velocity 当前机器人速度 (Current robot velocity)
   * @param goal_checker 指向任务正在使用的当前目标检查器的指针 (Pointer to the current goal checker
   * the task is utilizing)
   * @return 驾驶机器人的最佳命令 (The best command for the robot to drive)
   */
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &velocity,
      nav2_core::GoalChecker *goal_checker) = 0;

  /**
   * @brief 限制机器人的最大线速度 (Limits the maximum linear speed of the robot)
   * @param speed_limit 用绝对值表示（以m/s为单位）或者以百分比表示的最大机器人速度 (speed_limit
   * expressed in absolute value (in m/s) or in percentage from maximum robot speed)
   * @param percentage 如果为true，则以百分比设置速度限制；如果为false，则以绝对值设置 (Setting
   * speed limit in percentage if true or in absolute values in false case)
   */
  virtual void setSpeedLimit(const double &speed_limit, const bool &percentage) = 0;

  /**
   * @brief 如果任务退出后需要重置控制器的状态 (Reset the state of the controller if necessary after
   * task is exited)
   */
  virtual void reset() {}
};

}  // namespace nav2_core

#endif  // NAV2_CORE__CONTROLLER_HPP_
