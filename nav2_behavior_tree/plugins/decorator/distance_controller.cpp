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

#include <chrono>
#include <string>
#include <memory>
#include <cmath>

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/decorator/distance_controller.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief DistanceController 构造函数
 * 
 * @param[in] name 节点名称
 * @param[in] conf 节点配置
 *
 * @details
 * 该构造函数用于初始化 DistanceController 类的实例。
 */
DistanceController::DistanceController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf), // 初始化 DecoratorNode 基类
  distance_(1.0), // 设定默认距离为1.0
  global_frame_("map"), // 设定默认全局坐标系为 "map"
  robot_base_frame_("base_link"), // 设定默认机器人基座坐标系为 "base_link"
  first_time_(false) // 初始状态设为非第一次执行
{
  getInput("distance", distance_); // 获取输入参数 distance
  getInput("global_frame", global_frame_); // 获取输入参数 global_frame
  getInput("robot_base_frame", robot_base_frame_); // 获取输入参数 robot_base_frame
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node"); // 从黑板中获取共享指针 rclcpp::Node
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer"); // 从黑板中获取共享指针 tf2_ros::Buffer

  node_->get_parameter("transform_tolerance", transform_tolerance_); // 获取参数 transform_tolerance
}

/**
 * @brief tick 函数，用于执行节点操作
 *
 * @return BT::NodeStatus 返回节点状态
 *
 * @details
 * 此函数用于执行 DistanceController 类的操作，并根据操作结果返回节点状态。
 */
inline BT::NodeStatus DistanceController::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    // 如果当前状态为空闲，则重置起始位置，因为我们正在启动距离控制器的新迭代（从 IDLE 切换到 RUNNING）
    if (!nav2_util::getCurrentPose(
        start_pose_, *tf_, global_frame_, robot_base_frame_,
        transform_tolerance_))
    {
      RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
      return BT::NodeStatus::FAILURE;
    }
    first_time_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  // 确定自开始此次迭代以来已行驶的距离
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  // 获取欧几里得距离
  auto travelled = nav2_util::geometry_utils::euclidean_distance(
    start_pose_.pose, current_pose.pose);

  // 子节点在第一次执行时被 tick，每次越过阈值距离时被 tick。此外，一旦子节点开始运行，它将在每次完成之前进行 tick
  if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    travelled >= distance_)
  {
    first_time_ = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        if (!nav2_util::getCurrentPose(
            start_pose_, *tf_, global_frame_, robot_base_frame_,
            transform_tolerance_))
        {
          RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
          return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::DistanceController>("DistanceController");
}
