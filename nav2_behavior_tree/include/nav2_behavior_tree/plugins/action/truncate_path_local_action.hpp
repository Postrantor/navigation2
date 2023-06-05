// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_LOCAL_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_LOCAL_ACTION_HPP_

#include <limits>
#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

/**
 * @brief 一个 BT::ActionNodeBase 类，用于缩短机器人周围一定距离的路径
 * @brief A BT::ActionNodeBase class to shorten the path to some distance around the robot
 */
class TruncatePathLocal : public BT::ActionNodeBase
{
public:
  /**
   * @brief nav2_behavior_tree::TruncatePathLocal 构造函数
   * @brief A nav2_behavior_tree::TruncatePathLocal constructor
   * @param xml_tag_name 该节点的 XML 标签名
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT 节点配置
   * @param conf BT node configuration
   */
  TruncatePathLocal(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  /**
   * @brief 创建 BT 端口列表
   * @brief Creates a list of BT ports
   * @return 包含基本端口和节点特定端口的 BT::PortsList
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      // 输入端口：原始路径
      // Input port: Original Path
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Original Path"),
      // 输出端口：截断到机器人周围一定距离的路径
      // Output port: Path truncated to a certain distance around the robot
      BT::OutputPort<nav_msgs::msg::Path>(
        "output_path", "Path truncated to a certain distance around robot"),
      // 输入端口：前向距离
      // Input port: Distance in forward direction
      BT::InputPort<double>("distance_forward", 8.0, "Distance in forward direction"),
      // 输入端口：后向距离
      // Input port: Distance in backward direction
      BT::InputPort<double>("distance_backward", 4.0, "Distance in backward direction"),
      // 输入端口：机器人基座框架 ID
      // Input port: Robot base frame id
      BT::InputPort<std::string>("robot_frame", "base_link", "Robot base frame id"),
      // 输入端口：变换查找容差
      // Input port: Transform lookup tolerance
      BT::InputPort<double>("transform_tolerance", 0.2, "Transform lookup tolerance"),
      // 输入端口：手动指定的姿态（用于覆盖当前机器人姿态）
      // Input port: Manually specified pose to be used if overriding current robot pose
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "pose", "Manually specified pose to be used"
                "if overriding current robot pose"),
      // 输入端口：相对于位置距离查找路径姿态时的角距离权重
      // Input port: Weight of angular distance relative to positional distance when finding which path pose is closest to robot
      BT::InputPort<double>(
        "angular_distance_weight", 0.0,
        "Weight of angular distance relative to positional distance when finding which path "
        "pose is closest to robot. Not applicable on paths without orientations assigned"),
      // 输入端口：沿路径最大前向积分距离（从上次检测到的姿态开始），用于限制机器人最近姿态的搜索范围
      // Input port: Maximum forward integrated distance along the path (starting from the last detected pose) to bound the search for the closest pose to the robot
      BT::InputPort<double>(
        "max_robot_pose_search_dist", std::numeric_limits<double>::infinity(),
        "Maximum forward integrated distance along the path (starting from the last detected pose) "
        "to bound the search for the closest pose to the robot. When set to infinity (default), "
        "whole path is searched every time"),
    };
  }

private:
  /**
   * @brief 可选的 BT 操作中断覆盖
   * @brief The optional halt() override required by a BT action
   */
  void halt() override {}

  /**
   * @brief 执行 tick 的主要覆盖方法，由 BT 动作所需
   * @brief The main tick() override required by a BT action
   * @return 执行 tick 的状态
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 获取指定输入姿态或路径框架中的机器人姿态
   * @brief Get either specified input pose or robot pose in path frame
   * @param path_frame_id 路径的框架 ID
   * @param path_frame_id Frame ID of path
   * @param pose 输出姿态
   * @param pose Output pose
   * @return 如果成功，则返回 True
   * @return True if succeeded
   */
  bool getRobotPose(std::string path_frame_id, geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief 自定义姿态距离方法，除了空间距离外还考虑角距离（用于改进靠近尖点和循环的正确姿态选择）
   * @brief A custom pose distance method which takes angular distance into account in addition to spatial distance (to improve picking a correct pose near cusps and loops)
   * @param pose1 计算该姿态与 pose2 之间的距离
   * @param pose1 Distance is computed between this pose and pose2
   * @param pose2 计算该姿态与 pose1 之间的距离
   * @param pose2 Distance is computed between this pose and pose1
   * @param angular_distance_weight 角距离相对于空间距离的权重（1.0 表示 1 弧度的角距离对应 1 米的空间距离）
   * @param angular_distance_weight Weight of angular distance relative to spatial distance (1.0 means that 1 radian of angular distance corresponds to 1 meter of spatial distance)
   */
  static double poseDistance(
    const geometry_msgs::msg::PoseStamped & pose1,
    const geometry_msgs::msg::PoseStamped & pose2,
    const double angular_distance_weight);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path::_poses_type::iterator closest_pose_detection_begin_;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_LOCAL_ACTION_HPP_
