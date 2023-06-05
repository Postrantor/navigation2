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

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "nav2_behavior_tree/plugins/condition/goal_reached_condition.hpp"

namespace nav2_behavior_tree
{

// GoalReachedCondition 构造函数
// GoalReachedCondition constructor
GoalReachedCondition::GoalReachedCondition(
  const std::string & condition_name, // 条件节点名称，用于标识该条件节点
                                      // Condition node name, used to identify the condition node
  const BT::NodeConfiguration & conf) // 节点配置，包含行为树中的一些参数信息
// Node configuration, containing some parameter information in the behavior tree
: BT::ConditionNode(condition_name, conf), // 初始化 BT::ConditionNode 类
                                           // Initialize BT::ConditionNode class
  initialized_(false),                     // 初始化标志，表示节点是否已初始化
  // Initialization flag, indicating whether the node has been initialized
  global_frame_("map"),          // 全局坐标系名称，默认为 "map"
                                 // Global coordinate frame name, default is "map"
  robot_base_frame_("base_link") // 机器人基座坐标系名称，默认为 "base_link"
                                 // Robot base coordinate frame name, default is "base_link"
{
  getInput("global_frame", global_frame_); // 获取输入参数 "global_frame" 的值
                                           // Get the value of input parameter "global_frame"
  getInput("robot_base_frame", robot_base_frame_); // 获取输入参数 "robot_base_frame" 的值
    // Get the value of input parameter "robot_base_frame"
}

// GoalReachedCondition 析构函数
// GoalReachedCondition destructor
GoalReachedCondition::~GoalReachedCondition()
{
  cleanup(); // 清理资源
             // Clean up resources
}

// tick 函数，用于执行条件检查逻辑
// Tick function, used to execute condition check logic
BT::NodeStatus GoalReachedCondition::tick()
{
  if (!initialized_) { // 如果节点未初始化
                       // If the node is not initialized
    initialize();      // 初始化节点
                       // Initialize the node
  }

  if (isGoalReached()) {            // 如果已到达目标
                                    // If the goal has been reached
    return BT::NodeStatus::SUCCESS; // 返回成功状态
                                    // Return success status
  }
  return BT::NodeStatus::FAILURE; // 返回失败状态
                                  // Return failure status
}

// initialize 函数，用于初始化节点资源
// Initialize function, used to initialize node resources
void GoalReachedCondition::initialize()
{
  node_ =
    config().blackboard->get<rclcpp::Node::SharedPtr>("node"); // 从黑板中获取共享的 rclcpp::Node

  nav2_util::declare_parameter_if_not_declared(
    node_, "goal_reached_tol",
    rclcpp::ParameterValue(0.25)); // 声明参数 "goal_reached_tol"，如果未声明，则使用默认值 0.25
  node_->get_parameter_or<double>(
    "goal_reached_tol", goal_reached_tol_,
    0.25); // 获取参数 "goal_reached_tol" 的值，如果不存在，则使用默认值 0.25

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>(
    "tf_buffer"); // 从黑板中获取共享的 tf2_ros::Buffer

  node_->get_parameter(
    "transform_tolerance", transform_tolerance_); // 获取参数 "transform_tolerance" 的值

  initialized_ = true; // 设置初始化标志为 true
                       // Set initialization flag to true
}

// isGoalReached 函数，用于判断是否已到达目标
// IsGoalReached function, used to determine if the goal has been reached
bool GoalReachedCondition::isGoalReached()
{
  geometry_msgs::msg::PoseStamped current_pose; // 当前机器人位姿

  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_,
        transform_tolerance_)) { // 如果无法获取当前机器人位姿
    // If the current robot pose cannot be obtained
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }

  geometry_msgs::msg::PoseStamped goal; // 目标位姿
  getInput("goal", goal);               // 获取输入参数 "goal" 的值
                                        // Get the value of input parameter "goal"
  double dx =
    goal.pose.position.x - current_pose.pose.position.x; // 计算目标与当前位姿在 x 轴上的距离
    // Calculate the distance between the goal and the current pose on the x axis
  double dy =
    goal.pose.position.y - current_pose.pose.position.y; // 计算目标与当前位姿在 y 轴上的距离
    // Calculate the distance between the goal and the current pose on the y axis

  return (dx * dx + dy * dy) <=
         (goal_reached_tol_ *
          goal_reached_tol_); // 判断是否已到达目标，如果平方距离小于等于允许误差的平方，则认为已到达目标
    // Determine whether the goal has been reached. If the squared distance is less than or equal to the square of the allowable error, it is considered that the goal has been reached
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalReachedCondition>("GoalReached");
}
