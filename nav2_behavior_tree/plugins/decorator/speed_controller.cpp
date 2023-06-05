// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#include "nav2_util/geometry_utils.hpp"
#include <memory>
#include <string>
#include <vector>

#include "nav2_behavior_tree/plugins/decorator/speed_controller.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief SpeedController 构造函数 (Constructor for the SpeedController)
 *
 * @param name 节点名称 (Node name)
 * @param conf 节点配置 (Node configuration)
 */
SpeedController::SpeedController(const std::string & name, const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf), first_tick_(false), period_(1.0), min_rate_(0.1), max_rate_(1.0),
  min_speed_(0.0), max_speed_(0.5)
{
  // 从黑板中获取节点指针 (Get the node pointer from the blackboard)
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // 获取输入参数 (Get input parameters)
  getInput("min_rate", min_rate_);
  getInput("max_rate", max_rate_);
  getInput("min_speed", min_speed_);
  getInput("max_speed", max_speed_);

  // 检查速率参数是否合法 (Check if rate parameters are valid)
  if (min_rate_ <= 0.0 || max_rate_ <= 0.0) {
    std::string err_msg = "SpeedController node cannot have rate <= 0.0";
    RCLCPP_FATAL(node_->get_logger(), err_msg.c_str());
    throw BT::BehaviorTreeException(err_msg);
  }

  // 计算速率和速度的差值 (Calculate differences in rate and speed)
  d_rate_ = max_rate_ - min_rate_;
  d_speed_ = max_speed_ - min_speed_;

  // 获取 odom_topic 参数，默认为 "odom" (Get the odom_topic parameter, default is "odom")
  std::string odom_topic;
  node_->get_parameter_or("odom_topic", odom_topic, std::string("odom"));
  // 获取 OdomSmoother 指针 (Get the pointer to OdomSmoother)
  odom_smoother_ =
    config().blackboard->get<std::shared_ptr<nav2_util::OdomSmoother>>("odom_smoother");
}

/**
 * @brief tick() 函数，用于执行 SpeedController 节点的逻辑 (tick() function, used to execute the logic of the SpeedController node)
 *
 * @return BT::NodeStatus 返回节点状态 (Returns the node status)
 */
inline BT::NodeStatus SpeedController::tick()
{
  // 如果节点状态为空闲，则重置状态 (If the node status is IDLE, reset the state)
  if (status() == BT::NodeStatus::IDLE) {
    // 从黑板中获取目标信息 (Get goal information from the blackboard)
    config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", goals_);
    config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", goal_);
    period_ = 1.0 / max_rate_;
    start_ = node_->now();
    first_tick_ = true;
  }

  // 从黑板中获取当前目标信息 (Get current goal information from the blackboard)
  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  // 如果目标发生变化，重置状态并更新周期 (If the goal changes, reset the state and update the period)
  if (goal_ != current_goal || goals_ != current_goals) {
    period_ = 1.0 / max_rate_;
    start_ = node_->now();
    first_tick_ = true;
    goal_ = current_goal;
    goals_ = current_goals;
  }

  // 设置节点状态为运行中 (Set the node status to RUNNING)
  setStatus(BT::NodeStatus::RUNNING);

  // 计算经过的时间 (Calculate elapsed time)
  auto elapsed = node_->now() - start_;

  // 如果是第一次 tick，或者子节点正在运行，或者经过的时间大于等于周期，则执行子节点的 tick 函数 (If it's the first tick, or the child node is running, or the elapsed time is greater than or equal to the period, execute the tick function of the child node)
  if (
    first_tick_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    elapsed.seconds() >= period_) {
    first_tick_ = false;

    // 如果经过的时间大于等于周期，更新周期 (If the elapsed time is greater than or equal to the period, update the period)
    if (elapsed.seconds() >= period_) {
      updatePeriod();
      start_ = node_->now();
    }

    // 执行子节点的 tick 函数，并根据返回的状态进行相应处理 (Execute the tick function of the child node and handle it according to the returned status)
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
    case BT::NodeStatus::RUNNING:
      return BT::NodeStatus::RUNNING;

    case BT::NodeStatus::SUCCESS:
      return BT::NodeStatus::SUCCESS;

    case BT::NodeStatus::FAILURE:
    default:
      return BT::NodeStatus::FAILURE;
    }
  }

  // 返回节点状态 (Return the node status)
  return status();
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SpeedController>("SpeedController");
}
