// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2023 Samsung Research America
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

#ifndef NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
#define NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/behavior_tree_navigator.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

/*
上面的代码段为 `nav2_bt_navigator::BtNavigator` 类添加了参数列表说明，并使用中文形式对代码块中的每一行进行了尽可能详细的注释。这个类是一个使用行为树导航机器人到达目标位置的动作服务器。其中包含了构造函数、析构函数以及一系列生命周期回调函数。这些函数在节点的不同生命周期阶段执行，例如在配置、激活、停用、清理和关闭阶段。此外，代码还包括了与行为树相关的执行、里程计平滑器对象、度量反馈以及节点可以使用的旋转变换等成员变量。
*/

namespace nav2_bt_navigator {

/**
 * @class nav2_bt_navigator::BtNavigator
 * @brief 一个使用行为树使机器人导航到目标位置的动作服务器
 */
class BtNavigator : public nav2_util::LifecycleNode {
public:
  /**
   * @brief nav2_bt_navigator::BtNavigator 类的构造函数
   * @param options 控制节点创建的附加选项
   */
  explicit BtNavigator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  /**
   * @brief nav2_bt_navigator::BtNavigator 类的析构函数
   */
  ~BtNavigator();

protected:
  /**
   * @brief 配置成员变量
   *
   * 初始化导航器插件的动作服务器；订阅 "goal_sub"；并从 xml 文件构建行为树。
   * @param state 生命周期节点状态的引用
   * @return 成功或失败
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief 激活动作服务器
   * @param state 生命周期节点状态的引用
   * @return 成功或失败
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief 停用动作服务器
   * @param state 生命周期节点状态的引用
   * @return 成功或失败
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief 重置成员变量
   * @param state 生命周期节点状态的引用
   * @return 成功或失败
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief 当处于关闭状态时调用
   * @param state 生命周期节点状态的引用
   * @return 成功或失败
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  // 处理所有与 BT（行为树）相关的执行
  pluginlib::ClassLoader<nav2_core::NavigatorBase> class_loader_;
  std::vector<pluginlib::UniquePtr<nav2_core::NavigatorBase>> navigators_;
  nav2_core::NavigatorMuxer plugin_muxer_;

  // 里程计平滑器对象
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;

  // 反馈的度量
  std::string robot_frame_;
  std::string global_frame_;
  double transform_tolerance_;
  std::string odom_topic_;

  // 节点可以使用的旋转变换
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
