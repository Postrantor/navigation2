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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__DISTANCE_CONTROLLER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__DISTANCE_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief 一个 BT::DecoratorNode，每当机器人行进指定的距离时，都会触发其子节点
 *        (A BT::DecoratorNode that ticks its child every time the robot
 *         travels a specified distance)
 */
class DistanceController : public BT::DecoratorNode
{
public:
  /**
   * @brief nav2_behavior_tree::DistanceController 的构造函数
   *        (A constructor for nav2_behavior_tree::DistanceController)
   * @param name 用于此节点的 XML 标签的名称 (Name for the XML tag for this node)
   * @param conf BT 节点配置 (BT node configuration)
   */
  DistanceController(const std::string & name, const BT::NodeConfiguration & conf);

  /**
   * @brief 创建 BT 端口列表 (Creates list of BT ports)
   * @return 包含特定于节点的端口的 BT::PortsList (BT::PortsList Containing node-specific ports)
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 1.0, "Distance"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame")};
  }

private:
  /**
   * @brief 主要覆盖 BT 操作所需的 (The main override required by a BT action)
   * @return 执行 tick 的 BT::NodeStatus 状态 (BT::NodeStatus Status of tick execution)
   */
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_; // 声明共享指针类型的 ROS 节点 (Declare a shared pointer type ROS node)

  std::shared_ptr<tf2_ros::Buffer> tf_; // 声明共享指针类型的 tf2_ros::Buffer (Declare a shared pointer type tf2_ros::Buffer)
  double transform_tolerance_; // 变换容差 (Transformation tolerance)

  geometry_msgs::msg::PoseStamped start_pose_; // 起始姿态 (Start pose)
  double distance_; // 距离变量 (Distance variable)

  std::string global_frame_; // 全局坐标系名称 (Global frame name)
  std::string robot_base_frame_; // 机器人基座坐标系名称 (Robot base frame name)

  bool first_time_; // 判断是否为首次执行的布尔变量 (Boolean variable to determine if it is the first execution)
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__DISTANCE_CONTROLLER_HPP_
