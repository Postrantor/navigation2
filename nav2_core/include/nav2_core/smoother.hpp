// Copyright (c) 2021 RoboTech Vision
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

#ifndef NAV2_CORE__SMOOTHER_HPP_
#define NAV2_CORE__SMOOTHER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_core {

/**
 * @class Smoother
 * @brief 平滑器接口，作为所有平滑器插件的虚拟基类
 * @brief smoother interface that acts as a virtual base class for all smoother plugins
 */
class Smoother {
public:
  // 使用 std::shared_ptr 定义指向 nav2_core::Smoother 类型的智能指针别名
  // Define an alias for a smart pointer to nav2_core::Smoother type using std::shared_ptr
  using Ptr = std::shared_ptr<nav2_core::Smoother>;

  /**
   * @brief 虚拟析构函数
   * @brief Virtual destructor
   */
  virtual ~Smoother() {}

  /**
   * @brief 配置函数，需要在派生类中实现
   * @param[in] 生命周期节点的弱指针
   * @param[in] 名称
   * @param[in] tf2_ros::Buffer 的共享指针
   * @param[in] nav2_costmap_2d::CostmapSubscriber 的共享指针
   * @param[in] nav2_costmap_2d::FootprintSubscriber 的共享指针
   *
   * @brief Configure function to be implemented in derived classes
   * @param[in] Weak pointer to the lifecycle node
   * @param[in] Name
   * @param[in] Shared pointer to tf2_ros::Buffer
   * @param[in] Shared pointer to nav2_costmap_2d::CostmapSubscriber
   * @param[in] Shared pointer to nav2_costmap_2d::FootprintSubscriber
   */
  virtual void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
      std::string name,
      std::shared_ptr<tf2_ros::Buffer>,
      std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
      std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) = 0;

  /**
   * @brief 清理资源的方法
   * @brief Method to cleanup resources.
   */
  virtual void cleanup() = 0;

  /**
   * @brief 激活平滑器和执行中涉及的任何线程的方法
   * @brief Method to activate smoother and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief 停用平滑器和执行中涉及的任何线程的方法
   * @brief Method to deactivate smoother and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief 平滑给定路径的方法
   *
   * @param[in,out] 要平滑的路径
   * @param[in] 最大允许平滑持续时间
   * @return 如果平滑完成则为 true，否则如果被时间限制中断则为 false
   *
   * @brief Method to smooth given path
   *
   * @param[in,out] In-out path to be smoothed
   * @param[in] Maximum duration smoothing should take
   * @return If smoothing was completed (true) or interrupted by time limit (false)
   */
  virtual bool smooth(nav_msgs::msg::Path &path, const rclcpp::Duration &max_time) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__SMOOTHER_HPP_
