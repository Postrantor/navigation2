// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_CORE__GLOBAL_PLANNER_HPP_
#define NAV2_CORE__GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_core {

/**
 * @class GlobalPlanner
 * @brief 全局规划器的抽象接口，插件库需遵循此接口 (Abstract interface for global planners to adhere
 * to with pluginlib)
 */
class GlobalPlanner {
public:
  // 使用智能指针定义 Ptr 类型（Define Ptr type using smart pointer）
  using Ptr = std::shared_ptr<GlobalPlanner>;

  /**
   * @brief 虚拟析构函数 (Virtual destructor)
   */
  virtual ~GlobalPlanner() {}

  /**
   * @param  parent 用户节点的指针 (pointer to user's node)
   * @param  name 本规划器的名称 (The name of this planner)
   * @param  tf 指向 TF 缓冲区的指针 (A pointer to a TF buffer)
   * @param  costmap_ros 指向 costmap 的指针 (A pointer to the costmap)
   */
  virtual void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
      std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  /**
   * @brief 清理资源的方法，在关闭时使用 (Method to cleanup resources used on shutdown)
   */
  virtual void cleanup() = 0;

  /**
   * @brief 激活规划器及其执行所涉及的任何线程的方法 (Method to active planner and any threads
   * involved in execution)
   */
  virtual void activate() = 0;

  /**
   * @brief 停用规划器及其执行所涉及的任何线程的方法 (Method to deactivate planner and any threads
   * involved in execution)
   */
  virtual void deactivate() = 0;

  /**
   * @brief 从开始和结束目标创建计划的方法 (Method to create the plan from a starting and ending
   * goal)
   * @param start 机器人的起始姿态 (The starting pose of the robot)
   * @param goal  机器人的目标姿态 (The goal pose of the robot)
   * @return      从起点到终点的姿态序列，如果有的话 (The sequence of poses to get from start to
   * goal, if any)
   */
  virtual nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__GLOBAL_PLANNER_HPP_
