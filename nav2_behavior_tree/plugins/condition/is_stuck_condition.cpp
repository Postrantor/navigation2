// Copyright (c) 2018 Intel Corporation
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

#include <chrono>
#include <string>

#include "nav2_behavior_tree/plugins/condition/is_stuck_condition.hpp"

using namespace std::chrono_literals; // NOLINT

namespace nav2_behavior_tree
{

/**
 * @brief 构造函数，初始化 IsStuckCondition 类的对象 (Constructor, initializes the object of the IsStuckCondition class)
 *
 * @param condition_name 条件节点名称 (Name of the condition node)
 * @param conf 节点配置 (Node configuration)
 */
IsStuckCondition::IsStuckCondition(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf), is_stuck_(false), odom_history_size_(10),
  current_accel_(0.0), brake_accel_limit_(-10.0)
{
  // 从黑板中获取共享指针类型的节点 (Get the shared pointer type node from the blackboard)
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  // 创建回调组 (Create callback group)
  callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  // 将回调组添加到执行器中 (Add callback group to executor)
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  // 创建执行器线程 (Create executor thread)
  callback_group_executor_thread = std::thread([this]() { callback_group_executor_.spin(); });

  // 设置订阅选项 (Set subscription options)
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  // 创建里程计订阅 (Create odometry subscription)
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SystemDefaultsQoS(),
    std::bind(&IsStuckCondition::onOdomReceived, this, std::placeholders::_1), sub_option);

  // 输出调试信息 (Output debug information)
  RCLCPP_DEBUG(node_->get_logger(), "Initialized an IsStuckCondition BT node");

  // 输出等待里程计的提示信息 (Output waiting for odometry prompt)
  RCLCPP_INFO_ONCE(node_->get_logger(), "Waiting on odometry");
}

/**
 * @brief 析构函数，销毁 IsStuckCondition 类的对象 (Destructor, destroys the object of the IsStuckCondition class)
 */
IsStuckCondition::~IsStuckCondition()
{
  // 输出调试信息 (Output debug information)
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsStuckCondition BT node");
  // 取消执行器 (Cancel executor)
  callback_group_executor_.cancel();
  // 等待执行器线程结束 (Wait for executor thread to finish)
  callback_group_executor_thread.join();
}

/**
 * @brief 当接收到里程计消息时的回调函数 (Callback function when receiving odometer messages)
 *
 * @param msg 接收到的里程计消息 (Received odometer message)
 */
void IsStuckCondition::onOdomReceived(const typename nav_msgs::msg::Odometry::SharedPtr msg)
{
  // 输出接收到里程计的提示信息 (Output received odometer prompt)
  RCLCPP_INFO_ONCE(node_->get_logger(), "Got odometry");

  // 更新里程计历史记录 (Update odometer history)
  while (odom_history_.size() >= odom_history_size_) {
    odom_history_.pop_front();
  }

  odom_history_.push_back(*msg);

  // 更新状态 (Update states)
  updateStates();
}

/**
 * @brief 判断机器人是否卡住的条件节点 (Condition node to determine if the robot is stuck)
 *
 * @return BT::NodeStatus 成功检测到卡住条件返回成功，否则返回失败 (Return success if stuck condition is detected, otherwise return failure)
 */
BT::NodeStatus IsStuckCondition::tick()
{
  if (is_stuck_) {
    logStuck("Robot got stuck!");
    return BT::NodeStatus::SUCCESS; // Successfully detected a stuck condition
  }

  logStuck("Robot is free");
  return BT::NodeStatus::FAILURE; // Failed to detected a stuck condition
}

/**
 * @brief 记录机器人是否卡住的日志 (Log whether the robot is stuck or not)
 *
 * @param msg 需要记录的消息 (Message to be logged)
 */
void IsStuckCondition::logStuck(const std::string & msg) const
{
  static std::string prev_msg;

  if (msg == prev_msg) {
    return;
  }

  // 输出信息 (Output information)
  RCLCPP_INFO(node_->get_logger(), msg.c_str());
  prev_msg = msg;
}

/**
 * @brief 更新状态 (Update states)
 */
void IsStuckCondition::updateStates()
{
  // 计算近似加速度 (Calculate approximate acceleration)
  if (odom_history_.size() > 2) {
    auto curr_odom = odom_history_.end()[-1];
    double curr_time = static_cast<double>(curr_odom.header.stamp.sec);
    curr_time += (static_cast<double>(curr_odom.header.stamp.nanosec)) * 1e-9;

    auto prev_odom = odom_history_.end()[-2];
    double prev_time = static_cast<double>(prev_odom.header.stamp.sec);
    prev_time += (static_cast<double>(prev_odom.header.stamp.nanosec)) * 1e-9;

    double dt = curr_time - prev_time;
    double vel_diff =
      static_cast<double>(curr_odom.twist.twist.linear.x - prev_odom.twist.twist.linear.x);
    current_accel_ = vel_diff / dt;
  }

  // 判断是否卡住 (Determine if stuck)
  is_stuck_ = isStuck();
}

/**
 * @brief 判断机器人是否卡住 (Determine if the robot is stuck)
 *
 * @return bool 如果卡住返回 true，否则返回 false (Return true if stuck, otherwise return false)
 */
bool IsStuckCondition::isStuck()
{
  // 检测机器人是否撞到某物体，通过检查异常减速来判断 (Detect if the robot bumped into something by checking for abnormal deceleration)
  if (current_accel_ < brake_accel_limit_) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Current deceleration is beyond brake limit."
      " brake limit: %.2f, current accel: %.2f",
      brake_accel_limit_, current_accel_);

    return true;
  }

  return false;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsStuckCondition>("IsStuck");
}
