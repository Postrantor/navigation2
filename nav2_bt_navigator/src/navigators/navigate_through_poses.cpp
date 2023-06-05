// Copyright (c) 2021 Samsung Research
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

#include "nav2_bt_navigator/navigators/navigate_through_poses.hpp"

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace nav2_bt_navigator {

/*
该代码段是在ROS2项目中nav2_bt_navigator组件相关的代码。其中包含了两个函数，分别是configure和getDefaultBTFilepath。

configure函数用于配置导航器，设置黑板ID和平滑器对象。首先通过parent_node获取节点对象，并设置start_time_为0。然后判断是否设置了goals_blackboard_id和path_blackboard_id参数，如果没有则将其设置为默认值"goals"和"path"。最后设置平滑器对象并返回true表示配置成功。

getDefaultBTFilepath函数用于获取默认行为树文件路径。同样是通过parent_node获取节点对象。然后判断是否设置了default_nav_through_poses_bt_xml参数，如果没有则设置为默认路径。最后获取参数值并返回默认行为树文件路径。
*/
/**
 * @brief 配置导航器，设置黑板ID和平滑器对象
 * @param parent_node 导航器的父节点
 * @param odom_smoother 平滑器对象指针
 * @return bool 配置是否成功
 * @details 设置导航器的黑板ID和平滑器对象，返回配置是否成功的布尔值
 */
bool NavigateThroughPosesNavigator::configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) {
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();

  // 如果没有设置goals_blackboard_id参数，则将其设置为默认值"goals"
  if (!node->has_parameter("goals_blackboard_id")) {
    node->declare_parameter("goals_blackboard_id", std::string("goals"));
  }

  goals_blackboard_id_ = node->get_parameter("goals_blackboard_id").as_string();

  // 如果没有设置path_blackboard_id参数，则将其设置为默认值"path"
  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("path"));
  }

  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

  // 设置平滑器对象
  odom_smoother_ = odom_smoother;

  return true;
}

/**
 * @brief 获取默认行为树文件路径
 * @param parent_node 导航器的父节点
 * @return std::string 默认行为树文件路径
 * @details 获取默认的行为树文件路径，如果没有设置则使用默认路径
 */
std::string NavigateThroughPosesNavigator::getDefaultBTFilepath(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node) {
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  // 如果没有设置default_nav_through_poses_bt_xml参数，则将其设置为默认值
  if (!node->has_parameter("default_nav_through_poses_bt_xml")) {
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
    node->declare_parameter<std::string>(
        "default_nav_through_poses_bt_xml",
        pkg_share_dir + "/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml");
  }

  node->get_parameter("default_nav_through_poses_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

/*
上述代码是ROS2项目中nav2_bt_navigator组件相关的代码。其中NavigateThroughPosesNavigator类中的goalReceived函数用于接收并处理ActionT::Goal类型的目标，而goalCompleted函数则用于在导航完成时执行。

在goalReceived函数中，首先从目标中获取行为树文件名，并调用bt_action_server_对象的loadBehaviorTree函数加载该行为树文件。如果加载失败，则输出错误信息并返回false；否则，调用initializeGoalPoses函数对目标姿态进行初始化，并返回true。

在goalCompleted函数中，该函数为空实现，不做任何操作。
*/
/**
 * @brief
 * NavigateThroughPosesNavigator类中的goalReceived函数，用于接收并处理ActionT::Goal类型的目标。
 * @param goal 接收到的目标指针
 * @return bool 返回是否成功处理目标
 * @details
 * 从目标中获取行为树文件名，并加载该行为树文件。如果加载失败，则输出错误信息并返回false；否则，调用initializeGoalPoses函数对目标姿态进行初始化，并返回true。
 */
bool NavigateThroughPosesNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal) {
  auto bt_xml_filename = goal->behavior_tree;

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
        logger_, "Error loading XML file: %s. Navigation canceled.", bt_xml_filename.c_str());
    return false;
  }

  initializeGoalPoses(goal);

  return true;
}

/**
 * @brief NavigateThroughPosesNavigator类中的goalCompleted函数，用于在导航完成时执行。
 * @param result 结果指针
 * @param final_bt_status 行为树状态
 * @return void
 * @details 该函数为空实现，不做任何操作。
 */
void NavigateThroughPosesNavigator::goalCompleted(
    typename ActionT::Result::SharedPtr /*result*/,
    const nav2_behavior_tree::BtStatus /*final_bt_status*/) {}

/*
此代码段是在ROS2项目中nav2_bt_navigator组件相关的代码，用于在导航过程中循环执行的操作。该函数会根据当前机器人的位置和目标路径计算机器人到达目标点还需要的距离和时间，并发布反馈信息。

具体实现步骤如下：

1. 获取机器人当前位置和目标路径。
1. 查找机器人当前位置在全局路径上最近的点。
1. 计算机器人到达目标点还需要的距离。
1. 如果机器人速度大于1cm/s且距离目标点至少还有10cm，则计算机器人到达目标点还需要的时间。
1. 获取恢复次数、当前位置、导航时间和剩余目标点数量，并设置反馈信息。
1. 发布反馈信息。
*/
/**
 * @brief NavigateThroughPosesNavigator::onLoop()函数，用于在导航过程中循环执行的操作
 * @param 无
 * @details 根据当前机器人的位置和目标路径计算机器人到达目标点还需要的距离和时间，并发布反馈信息。
 */
void NavigateThroughPosesNavigator::onLoop() {
  using namespace nav2_util::geometry_utils;  // NOLINT

  // action server feedback (pose, duration of task,
  // number of recoveries, and distance remaining to goal, etc)
  auto feedback_msg = std::make_shared<ActionT::Feedback>();

  auto blackboard = bt_action_server_->getBlackboard();

  Goals goal_poses;
  blackboard->get<Goals>(goals_blackboard_id_, goal_poses);

  if (goal_poses.size() == 0) {  // 如果目标点为空，则直接返回
    bt_action_server_->publishFeedback(feedback_msg);
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  nav2_util::getCurrentPose(
      current_pose, *feedback_utils_.tf, feedback_utils_.global_frame, feedback_utils_.robot_frame,
      feedback_utils_.transform_tolerance);

  try {
    // Get current path points
    nav_msgs::msg::Path current_path;
    blackboard->get<nav_msgs::msg::Path>(path_blackboard_id_, current_path);

    // Find the closest pose to current pose on global path
    auto find_closest_pose_idx = [&current_pose,
                                  &current_path]() {  // 查找机器人当前位置在全局路径上最近的点
      size_t closest_pose_idx = 0;
      double curr_min_dist = std::numeric_limits<double>::max();
      for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
        double curr_dist = nav2_util::geometry_utils::euclidean_distance(
            current_pose, current_path.poses[curr_idx]);
        if (curr_dist < curr_min_dist) {
          curr_min_dist = curr_dist;
          closest_pose_idx = curr_idx;
        }
      }
      return closest_pose_idx;
    };

    // Calculate distance on the path
    double distance_remaining = nav2_util::geometry_utils::calculate_path_length(
        current_path, find_closest_pose_idx());  // 计算机器人到达目标点还需要的距离

    // Default value for time remaining
    rclcpp::Duration estimated_time_remaining = rclcpp::Duration::from_seconds(0.0);

    // Get current speed
    geometry_msgs::msg::Twist current_odom = odom_smoother_->getTwist();
    double current_linear_speed =
        std::hypot(current_odom.linear.x, current_odom.linear.y);  // 获取当前机器人的线速度

    // Calculate estimated time taken to goal if speed is higher than 1cm/s
    // and at least 10cm to go
    if ((std::abs(current_linear_speed) > 0.01) &&
        (distance_remaining >
         0.1)) {  // 如果机器人速度大于1cm/s且距离目标点至少还有10cm，则计算机器人到达目标点还需要的时间
      estimated_time_remaining =
          rclcpp::Duration::from_seconds(distance_remaining / std::abs(current_linear_speed));
    }

    feedback_msg->distance_remaining = distance_remaining;  // 设置反馈信息中剩余距离
    feedback_msg->estimated_time_remaining =
        estimated_time_remaining;  // 设置反馈信息中预计剩余时间
  } catch (...) {
    // Ignore
  }

  int recovery_count = 0;
  blackboard->get<int>("number_recoveries", recovery_count);    // 获取恢复次数
  feedback_msg->number_of_recoveries = recovery_count;          // 设置反馈信息中恢复次数
  feedback_msg->current_pose = current_pose;                    // 设置反馈信息中当前位置
  feedback_msg->navigation_time = clock_->now() - start_time_;  // 设置反馈信息中导航时间
  feedback_msg->number_of_poses_remaining = goal_poses.size();  // 设置反馈信息中剩余目标点数量

  bt_action_server_->publishFeedback(feedback_msg);             // 发布反馈信息
}

/*
这段代码是 NavigateThroughPosesNavigator 类中的两个函数。onPreempt
函数用于处理目标预撤销请求，如果当前正在执行的行为树与待处理目标的行为树相同，则接受该目标；如果当前正在执行的行为树是默认行为树且待处理目标没有指定行为树，则接受该目标。否则拒绝该目标，并终止待处理目标。

initializeGoalPoses
函数用于初始化目标姿态，如果目标中包含姿态，则打印日志并更新黑板上的目标姿态；重置新行为反馈的状态；将恢复次数设置为0。
*/
/**
 * @brief NavigateThroughPosesNavigator类的onPreempt函数，用于处理目标预撤销请求
 * @param goal 目标指针
 * @details
 * 如果当前正在执行的行为树与待处理目标的行为树相同，则接受该目标；如果当前正在执行的行为树是默认行为树且待处理目标没有指定行为树，则接受该目标。否则拒绝该目标，并终止待处理目标。
 */
void NavigateThroughPosesNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal) {
  RCLCPP_INFO(logger_, "Received goal preemption request");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
      (goal->behavior_tree.empty() &&
       bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename())) {
    // 如果待处理目标请求的行为树与当前正在执行的行为树相同，或者待处理目标没有指定行为树且当前正在执行的行为树是默认行为树，则接受该目标
    initializeGoalPoses(bt_action_server_->acceptPendingGoal());
  } else {
    RCLCPP_WARN(
        logger_,
        "Preemption request was rejected since the requested BT XML file is not the same "
        "as the one that the current goal is executing. Preemption with a new BT is invalid "
        "since it would require cancellation of the previous goal instead of true preemption."
        "\nCancel the current goal and send a new action request if you want to use a "
        "different BT XML file. For now, continuing to track the last goal until completion.");
    bt_action_server_->terminatePendingGoal();  // 否则拒绝该目标，并终止待处理目标
  }
}

/**
 * @brief NavigateThroughPosesNavigator类的initializeGoalPoses函数，用于初始化目标姿态
 * @param goal 目标指针
 * @details
 * 如果目标中包含姿态，则打印日志并更新黑板上的目标姿态；重置新行为反馈的状态；将恢复次数设置为0。
 */
void NavigateThroughPosesNavigator::initializeGoalPoses(ActionT::Goal::ConstSharedPtr goal) {
  if (goal->poses.size() > 0) {
    RCLCPP_INFO(
        logger_, "Begin navigating from current location through %zu poses to (%.2f, %.2f)",
        goal->poses.size(), goal->poses.back().pose.position.x, goal->poses.back().pose.position.y);
  }

  // Reset state for new action feedback
  start_time_ = clock_->now();                   // 重置新行为反馈的状态
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<int>("number_recoveries", 0);  // NOLINT 将恢复次数设置为0

  // Update the goal pose on the blackboard 更新黑板上的目标姿态
  blackboard->set<Goals>(goals_blackboard_id_, goal->poses);
}

}  // namespace nav2_bt_navigator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_bt_navigator::NavigateThroughPosesNavigator, nav2_core::NavigatorBase)
