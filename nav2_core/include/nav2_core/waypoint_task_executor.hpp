// Copyright (c) 2020 Fetullah Atas
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

#ifndef NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
#define NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
#pragma once

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_core {
/**
 * @brief Base class for creating a plugin in order to perform a specific task at waypoint arrivals.
 *
 */
// 基类，用于创建插件以便在航点到达时执行特定任务。
class WaypointTaskExecutor {
public:
  /**
   * @brief Construct a new Simple Task Execution At Waypoint Base object
   *
   */
  // 构造一个新的简单任务执行航点基对象
  WaypointTaskExecutor() {}

  /**
   * @brief Destroy the Simple Task Execution At Waypoint Base object
   *
   */
  // 销毁简单任务执行航点基对象
  virtual ~WaypointTaskExecutor() {}

  /**
   * @brief Override this to setup your pub, sub or any ros services that you will use in the
   * plugin.
   *
   * @param parent parent node that plugin will be created within(for an example see
   * nav_waypoint_follower)
   * @param plugin_name plugin name comes from parameters in yaml file
   */
  // 重写此方法以设置您将在插件中使用的pub、sub或任何ros服务。
  // 覆盖此方法以设置你要在插件中使用的发布器、订阅器或任何 ROS 服务。
  //
  // @param parent 插件将在其中创建的父节点（例如，请参见 nav_waypoint_follower）
  // @param plugin_name 插件名称来自yaml文件中的参数
  virtual void initialize(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& plugin_name) = 0;

  /**
   * @brief Override this to define the body of your task that you would like to execute once the
   * robot arrived to waypoint
   *
   * @param curr_pose current pose of the robot
   * @param curr_waypoint_index current waypoint, that robot just arrived
   * @return true if task execution was successful
   * @return false if task execution failed
   */
  // 重写此方法以定义您希望在机器人到达航点后执行的任务主体。
  // 覆盖此方法以定义你希望在机器人抵达航点后执行的任务主体。
  //
  // @param curr_pose 机器人的当前姿态
  // @param curr_waypoint_index 机器人刚刚到达的当前航点
  // @return 如果任务执行成功，则返回true
  // @return 如果任务执行失败，则返回false
  virtual bool processAtWaypoint(
      const geometry_msgs::msg::PoseStamped& curr_pose, const int& curr_waypoint_index) = 0;
};
}  // namespace nav2_core
#endif  // NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
