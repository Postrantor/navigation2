// Copyright (c) 2021 Samsung Research America
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_behavior_tree
{

/**
 * @class RemovePassedGoals
 * @brief 一个基于行为树的 ActionNodeBase 类，用于移除已经通过的目标点。 (A Behavior Tree ActionNodeBase class for removing passed goals.)
 */
class RemovePassedGoals : public BT::ActionNodeBase
{
public:
  // 定义一个类型别名，表示一系列的目标点。(Define a type alias representing a series of goal points.)
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  /**
   * @brief 构造函数。(Constructor.)
   * @param xml_tag_name XML 标签名称。(XML tag name.)
   * @param conf 节点配置。(Node configuration.)
   */
  RemovePassedGoals(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  /**
   * @brief 提供的端口列表。(Provided ports list.)
   * @return 端口列表。(Ports list.)
   */
  static BT::PortsList providedPorts()
  {
    return {
      // 输入端口：原始目标点，用于从中移除途经点。(Input port: Original goals to remove viapoints from.)
      BT::InputPort<Goals>("input_goals", "Original goals to remove viapoints from"),
      // 输出端口：移除已通过途经点后的目标点。(Output port: Goals with passed viapoints removed.)
      BT::OutputPort<Goals>("output_goals", "Goals with passed viapoints removed"),
      // 输入端口：到达目标点的半径，用于判断是否应该移除该目标点。(Input port: radius to goal for it to be considered for removal.)
      BT::InputPort<double>("radius", 0.5, "radius to goal for it to be considered for removal"),
      // 输入端口：全局坐标系。(Input port: Global frame.)
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      // 输入端口：机器人基座的坐标系。(Input port: Robot base frame.)
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
    };
  }

private:
  /**
   * @brief 停止行为。(Halt behavior.)
   */
  void halt() override {}

  /**
   * @brief 执行行为。(Execute behavior.)
   * @return 节点状态。(Node status.)
   */
  BT::NodeStatus tick() override;

  // 到达途经点的半径。(Radius to reach the viapoint.)
  double viapoint_achieved_radius_;
  // 机器人基座的坐标系。(Robot base frame.)
  std::string robot_base_frame_;
  // 全局坐标系。(Global frame.)
  std::string global_frame_;
  // 坐标变换容差。(Coordinate transformation tolerance.)
  double transform_tolerance_;
  // 坐标变换缓冲区。(Coordinate transformation buffer.)
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
