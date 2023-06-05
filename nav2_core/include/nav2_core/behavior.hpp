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

#ifndef NAV2_CORE__BEHAVIOR_HPP_
#define NAV2_CORE__BEHAVIOR_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_core {

// 定义一个枚举类，表示代价地图信息类型
// Define an enumeration class, representing costmap information types
enum class CostmapInfoType { NONE = 0, LOCAL = 1, GLOBAL = 2, BOTH = 3 };

/**
 * @class Behavior
 * @brief 抽象接口，用于插件库中的行为遵循
 *        Abstract interface for behaviors to adhere to with pluginlib
 */
class Behavior {
public:
  // 使用 shared_ptr 定义一个 Ptr 类型
  // Use shared_ptr to define a Ptr type
  using Ptr = std::shared_ptr<Behavior>;

  /**
   * @brief 虚析构函数
   *        Virtual destructor
   */
  virtual ~Behavior() {}

  /**
   * @param  parent 指向用户节点的指针
   * @param  name 此规划器的名称
   * @param  tf 指向 TF 缓冲区的指针
   * @param  costmap_ros 指向 costmap 的指针
   *
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   * @param  costmap_ros A pointer to the costmap
   */
  virtual void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
      const std::string& name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> local_collision_checker,
      std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> global_collision_checker) = 0;

  /**
   * @brief 清理在关闭时使用的资源的方法
   *        Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief 激活行为及其执行中涉及的任何线程的方法
   *        Method to active Behavior and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief 停用行为及其执行中涉及的任何线程的方法
   *        Method to deactive Behavior and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief 确定所需代价地图信息的方法
   *        Method to determine the required costmap info
   * @return 所需的代价地图资源
   *         costmap resources needed
   */
  virtual CostmapInfoType getResourceInfo() = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__BEHAVIOR_HPP_
