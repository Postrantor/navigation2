// Copyright (c) 2021-2023 Samsung Research
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

#ifndef NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_TO_POSE_HPP_
#define NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_TO_POSE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/behavior_tree_navigator.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

/*
这段代码是一个用于导航到指定位姿的导航器，它继承自 `nav2_core::BehaviorTreeNavigator` 类，并使用
`nav2_msgs::action::NavigateToPose` 消息类型。其中包含了一些函数：

- `configure()`：配置状态转换以配置导航器的状态；
- `cleanup()`：清理状态转换以释放分配的内存；
- `onGoalPoseReceived()`：订阅和回调处理从 rviz 发布的基于话题的目标；
- `getName()`：获取此导航器的操作名称；
- `getDefaultBTFilepath()`：获取导航器的默认行为树；
- `goalReceived()`：当 BT 操作服务器接收到新目标时调用的回调函数；
- `onLoop()`：定义通过 BT 迭代执行的回调函数；
- `onPreempt()`：当请求取消时调用的回调函数；
- `goalCompleted()`：当操作完成时调用的回调函数，可以填充操作结果消息或指示此操作已完成；
- `initializeGoalPose()`：在黑板上初始化目标位姿。

其中，`configure()`、`cleanup()`、`onGoalPoseReceived()`、`getDefaultBTFilepath()`、`goalReceived()`、`onLoop()`、`onPreempt()`
和 `goalCompleted()`
都是重写了基类中的虚函数，并在此基础上实现了导航器的具体功能。`initializeGoalPose()`
则是用于初始化目标位姿的函数。
*/

namespace nav2_bt_navigator {

/**
 * @class NavigateToPoseNavigator
 * @brief 导航到指定位姿的导航器
 */
class NavigateToPoseNavigator
    : public nav2_core::BehaviorTreeNavigator<nav2_msgs::action::NavigateToPose> {
public:
  using ActionT = nav2_msgs::action::NavigateToPose;

  /**
   * @brief NavigateToPoseNavigator 的构造函数
   */
  NavigateToPoseNavigator() : BehaviorTreeNavigator() {}

  /**
   * @brief 配置状态转换以配置导航器的状态
   * @param node 生命周期节点的弱指针
   * @param odom_smoother 获取当前平滑机器人速度的对象
   * @return bool 配置是否成功
   */
  bool configure(
      rclcpp_lifecycle::LifecycleNode::WeakPtr node,
      std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  /**
   * @brief 清理状态转换以释放分配的内存
   * @return bool 清理是否成功
   */
  bool cleanup() override;

  /**
   * @brief 订阅和回调处理从 rviz 发布的基于话题的目标
   * @param pose 通过话题接收到的位姿
   */
  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

  /**
   * @brief 获取此导航器的操作名称
   * @return string 操作服务器的名称
   */
  std::string getName() { return std::string("navigate_to_pose"); }

  /**
   * @brief 获取导航器的默认 BT
   * @param node 生命周期节点的弱指针
   * @return string 默认 XML 的文件路径
   */
  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  /**
   * @brief 当 BT 操作服务器接收到新目标时调用的回调函数
   * 可以用于检查目标是否有效，并在黑板上放置依赖于接收到的目标的值
   * @param goal Action 模板的目标消息
   * @return bool 是否成功接收并处理目标
   */
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief 定义通过 BT 迭代执行的回调函数
   * 可以用于发布操作反馈
   */
  void onLoop() override;

  /**
   * @brief 当请求取消时调用的回调函数
   * @param goal Action 模板的目标消息
   */
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief 当操作完成时调用的回调函数，可以填充操作结果消息或指示此操作已完成。
   * @param result 要填充的 Action 模板结果消息
   * @param final_bt_status 在填充结果时可能会引用的行为树执行的结果状态
   */
  void goalCompleted(
      typename ActionT::Result::SharedPtr result,
      const nav2_behavior_tree::BtStatus final_bt_status) override;

  /**
   * @brief 在黑板上初始化目标位姿
   * @param goal 要处理的 Action 模板目标消息
   */
  void initializeGoalPose(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  std::string goal_blackboard_id_;
  std::string path_blackboard_id_;

  // 平滑里程计对象
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_TO_POSE_HPP_
