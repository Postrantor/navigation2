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

#ifndef NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_THROUGH_POSES_HPP_
#define NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_THROUGH_POSES_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/behavior_tree_navigator.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

/*
该代码段是一个 C++ 类，名为 NavigateThroughPosesNavigator，继承自 nav2_core::BehaviorTreeNavigator
类。该类实现了导航到一系列中间姿态的功能，是 ROS2 项目中 nav2_bt_navigator 组件的一部分。

该类包含了以下公共成员函数：

- NavigateThroughPosesNavigator()：构造函数。
- bool configure(rclcpp_lifecycle::LifecycleNode::WeakPtr node,
std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override：配置状态转换以配置导航器的状态。
- std::string getName() override：获取此导航器的操作名称。
- std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node)
override：获取导航器的默认 BT。
- bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override：当 BT
操作服务器接收到新目标时调用的回调函数。
- void onLoop() override：定义通过 BT 执行一次迭代时发生的执行的回调函数。
- void onPreempt(ActionT::Goal::ConstSharedPtr goal) override：当请求取消时调用的回调函数。
- void goalCompleted(typename ActionT::Result::SharedPtr result, const nav2_behavior_tree::BtStatus
final_bt_status) override：当操作完成时调用的回调函数，可以填充操作结果消息或指示此操作已完成。
- void initializeGoalPoses(ActionT::Goal::ConstSharedPtr goal)：在黑板上初始化目标姿态。

其中，configure() 函数、getDefaultBTFilepath() 函数、goalReceived() 函数、onLoop() 函数、onPreempt()
函数和 goalCompleted() 函数都是该类的回调函数，用于在特定事件发生时执行相应的操作。getName()
函数返回此导航器的操作名称。initializeGoalPoses() 函数用于在黑板上初始化目标姿态。

该类还包含了以下私有成员变量：

- rclcpp::Time start_time_：开始时间。
- std::string goals_blackboard_id_：目标黑板 ID。
- std::string path_blackboard_id_：路径黑板 ID。
- std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_：平滑里程计对象。
*/

namespace nav2_bt_navigator {

/**
 * @class NavigateThroughPosesNavigator
 * @brief 一个导航器，用于导航到一系列中间姿态
 */
class NavigateThroughPosesNavigator
    : public nav2_core::BehaviorTreeNavigator<nav2_msgs::action::NavigateThroughPoses> {
public:
  using ActionT = nav2_msgs::action::NavigateThroughPoses;
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  /**
   * @brief NavigateThroughPosesNavigator的构造函数
   */
  NavigateThroughPosesNavigator() : BehaviorTreeNavigator() {}

  /**
   * @brief 配置状态转换以配置导航器的状态
   * @param node 生命周期节点的弱指针
   * @param odom_smoother 用于获取当前平滑机器人速度的对象
   * @return bool 如果配置成功则返回true
   */
  bool configure(
      rclcpp_lifecycle::LifecycleNode::WeakPtr node,
      std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  /**
   * @brief 获取此导航器的操作名称
   * @return string 操作服务器的名称
   */
  std::string getName() override { return std::string("navigate_through_poses"); }

  /**
   * @brief 获取导航器的默认BT
   * @param node 生命周期节点的弱指针
   * @return string 默认XML的文件路径
   */
  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  /**
   * @brief 当BT操作服务器接收到新目标时调用的回调函数
   * 可用于检查目标是否有效，并在黑板上放置依赖于接收到的目标的值
   * @param goal 操作模板的目标消息
   * @return bool 如果成功接收目标以进行处理，则返回true
   */
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief 定义通过BT执行一次迭代时发生的执行的回调函数
   * 可用于发布操作反馈
   */
  void onLoop() override;

  /**
   * @brief 当请求取消时调用的回调函数
   * @param goal 操作模板的目标消息
   */
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief 当操作完成时调用的回调函数，可以填充操作结果消息或指示此操作已完成。
   * @param result 要填充的操作模板结果消息
   * @param final_bt_status 可以在填充结果时引用的行为树执行的结果状态。
   */
  void goalCompleted(
      typename ActionT::Result::SharedPtr result,
      const nav2_behavior_tree::BtStatus final_bt_status) override;

  /**
   * @brief 在黑板上初始化目标姿态
   * @param goal 操作模板的目标消息
   */
  void initializeGoalPoses(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;
  std::string goals_blackboard_id_;
  std::string path_blackboard_id_;

  // 平滑里程计对象
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_THROUGH_POSES_HPP_
