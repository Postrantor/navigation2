// Copyright (c) 2019 Intel Corporation
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

#include <memory>
#include <string>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "nav2_behavior_tree/plugins/condition/distance_traveled_condition.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 构造函数，用于初始化 DistanceTraveledCondition 类的对象 (Constructor for initializing the DistanceTraveledCondition class object)
 *
 * @param condition_name 条件节点名称 (Condition node name)
 * @param conf 节点配置 (Node configuration)
 */
DistanceTraveledCondition::DistanceTraveledCondition(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf), distance_(1.0), transform_tolerance_(0.1),
  global_frame_("map"), robot_base_frame_("base_link")
{
  // 获取输入参数 (Get input parameters)
  getInput("distance", distance_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);

  // 从黑板中获取节点和 tf 缓冲区 (Get node and tf buffer from the blackboard)
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  // 获取 transform_tolerance 参数 (Get the transform_tolerance parameter)
  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

/**
 * @brief tick 函数，用于执行距离检查逻辑 (Tick function for executing distance check logic)
 *
 * @return 返回节点状态 (Returns the node status)
 */
BT::NodeStatus DistanceTraveledCondition::tick()
{
  // 如果状态为 IDLE，则获取机器人当前位置 (If status is IDLE, get the current robot pose)
  if (status() == BT::NodeStatus::IDLE) {
    if (!nav2_util::getCurrentPose(
          start_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
      RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    }
    return BT::NodeStatus::FAILURE;
  }

  // 获取当前机器人位置 (Get the current robot pose)
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  // 计算欧几里得距离 (Calculate Euclidean distance)
  auto travelled =
    nav2_util::geometry_utils::euclidean_distance(start_pose_.pose, current_pose.pose);

  // 如果已经行驶的距离小于给定距离，则返回 FAILURE (If the travelled distance is less than the given distance, return FAILURE)
  if (travelled < distance_) {
    return BT::NodeStatus::FAILURE;
  }

  // 更新起始位置 (Update the start pose)
  start_pose_ = current_pose;

  // 返回成功状态 (Return success status)
  return BT::NodeStatus::SUCCESS;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::DistanceTraveledCondition>("DistanceTraveled");
}
