// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__DISTANCE_TRAVELED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__DISTANCE_TRAVELED_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

/**
 * @brief 一个BT::ConditionNode，当机器人每次行进指定距离时返回SUCCESS，否则返回FAILURE
 *        (A BT::ConditionNode that returns SUCCESS every time the robot
 *        travels a specified distance and FAILURE otherwise)
 */
class DistanceTraveledCondition : public BT::ConditionNode
{
public:
  /**
   * @brief nav2_behavior_tree::DistanceTraveledCondition的构造函数
   *        (A constructor for nav2_behavior_tree::DistanceTraveledCondition)
   * @param condition_name 此节点的XML标签名称 (Name for the XML tag for this node)
   * @param conf BT节点配置 (BT node configuration)
   */
  DistanceTraveledCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

  // 删除默认构造函数 (Delete the default constructor)
  DistanceTraveledCondition() = delete;

  /**
   * @brief 主要覆盖BT操作所需的方法 (The main override required by a BT action)
   * @return BT::NodeStatus tick执行状态 (Status of tick execution)
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 创建BT端口列表 (Creates list of BT ports)
   * @return BT::PortsList 包含特定于节点的端口 (Containing node-specific ports)
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 1.0, "Distance"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame")};
  }

private:
  rclcpp::Node::SharedPtr node_;        // ROS2节点 (ROS2 Node)
  std::shared_ptr<tf2_ros::Buffer> tf_; // tf2缓冲区 (tf2 buffer)

  geometry_msgs::msg::PoseStamped start_pose_; // 开始姿势 (Start pose)

  double distance_;              // 距离 (Distance)
  double transform_tolerance_;   // 变换容差 (Transform tolerance)
  std::string global_frame_;     // 全局帧 (Global frame)
  std::string robot_base_frame_; // 机器人基座帧 (Robot base frame)
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__DISTANCE_TRAVELED_CONDITION_HPP_
