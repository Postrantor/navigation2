// Copyright (c) 2018 Samsung Research America
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
// limitations under the License. Reserved.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/behavior.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

#ifndef NAV2_BEHAVIORS__BEHAVIOR_SERVER_HPP_
#define NAV2_BEHAVIORS__BEHAVIOR_SERVER_HPP_

namespace behavior_server {

/**
 * @class behavior_server::BehaviorServer
 * @brief 一个托管行为插件映射的服务器
 */
class BehaviorServer : public nav2_util::LifecycleNode {
public:
  /**
   * @brief 构造函数，用于创建 behavior_server::BehaviorServer 对象
   * @param options 控制节点创建的附加选项
   */
  explicit BehaviorServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief 析构函数，用于销毁 behavior_server::BehaviorServer 对象
   */
  ~BehaviorServer();

protected:
  /**
   * @brief 从参数文件中加载行为插件
   * @return 如果成功加载插件，则返回 true
   */
  bool loadBehaviorPlugins();

  /**
   * @brief 配置行为插件
   */
  void configureBehaviorPlugins();

  /**
   * @brief 为行为插件设置资源
   */
  void setupResourcesForBehaviorPlugins();

  /**
   * @brief 配置生命周期服务器
   * @param state 生命周期状态
   * @return 生命周期回调返回值
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief 激活生命周期服务器
   * @param state 生命周期状态
   * @return 生命周期回调返回值
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief 停用生命周期服务器
   * @param state 生命周期状态
   * @return 生命周期回调返回值
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief 清理生命周期服务器
   * @param state 生命周期状态
   * @return 生命周期回调返回值
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief 关闭生命周期服务器
   * @param state 生命周期状态
   * @return 生命周期回调返回值
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  // 插件
  pluginlib::ClassLoader<nav2_core::Behavior> plugin_loader_;         // 插件加载器
  std::vector<pluginlib::UniquePtr<nav2_core::Behavior>> behaviors_;  // 行为插件列表
  std::vector<std::string> default_ids_;     // 默认行为插件 ID 列表
  std::vector<std::string> default_types_;   // 默认行为插件类型列表
  std::vector<std::string> behavior_ids_;    // 行为插件 ID 列表
  std::vector<std::string> behavior_types_;  // 行为插件类型列表

  // 工具
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> local_costmap_sub_;  // 本地代价地图订阅器
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> local_footprint_sub_;  // 本地足迹订阅器
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker>
      local_collision_checker_;  // 本地代价地图碰撞检测器

  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> global_costmap_sub_;  // 全局代价地图订阅器
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> global_footprint_sub_;  // 全局足迹订阅器
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker>
      global_collision_checker_;         // 全局代价地图碰撞检测器

  std::shared_ptr<tf2_ros::Buffer> tf_;  // TF2 缓存指针
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;  // TF2 变换监听器指针
};

}  // namespace behavior_server

#endif  // NAV2_BEHAVIORS__BEHAVIOR_SERVER_HPP_
