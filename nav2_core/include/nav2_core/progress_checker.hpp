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

#ifndef NAV2_CORE__PROGRESS_CHECKER_HPP_
#define NAV2_CORE__PROGRESS_CHECKER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_core {
/**
 * @class nav2_core::ProgressChecker
 * @brief 这个类定义了用于检查机器人位置的插件接口，以确保它实际上是朝着目标前进的。
 * This class defines the plugin interface used to check the
 * position of the robot to make sure that it is actually progressing
 * towards a goal.
 */
class ProgressChecker {
public:
  // 定义一个智能指针类型，用于表示 ProgressChecker 类的对象
  // Define a smart pointer type for representing ProgressChecker class objects
  typedef std::shared_ptr<nav2_core::ProgressChecker> Ptr;

  // 虚析构函数，允许派生类正确析构
  // Virtual destructor, allowing derived classes to destruct properly
  virtual ~ProgressChecker() {}

  /**
   * @brief 初始化 ProgressChecker 的参数
   * Initialize parameters for ProgressChecker
   * @param parent 节点指针
   * @param parent Node pointer
   * @param plugin_name 插件名称
   * @param plugin_name Plugin name
   */
  virtual void initialize(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& plugin_name) = 0;

  /**
   * @brief 检查机器人相对于先前姿态是否移动
   * Checks if the robot has moved compared to previous pose
   * @param current_pose 机器人的当前姿态
   * @param current_pose Current pose of the robot
   * @return 如果取得进展，则返回 True
   * @return True if progress is made
   */
  virtual bool check(geometry_msgs::msg::PoseStamped& current_pose) = 0;

  /**
   * @brief 调用时重置类状态
   * Reset class state upon calling
   */
  virtual void reset() = 0;
};
}  // namespace nav2_core

#endif  // NAV2_CORE__PROGRESS_CHECKER_HPP_
