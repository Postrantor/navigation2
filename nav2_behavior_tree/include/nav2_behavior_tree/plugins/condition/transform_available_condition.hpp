// Copyright (c) 2020 Samsung Research America
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TRANSFORM_AVAILABLE_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TRANSFORM_AVAILABLE_CONDITION_HPP_

#include <atomic>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

/**
 * @brief 一个BT::ConditionNode，如果两个指定的帧之间有有效的转换，则返回SUCCESS，否则返回FAILURE
 *        (A BT::ConditionNode that returns SUCCESS if there is a valid transform
 *        between two specified frames and FAILURE otherwise)
 */
class TransformAvailableCondition : public BT::ConditionNode
{
public:
  /**
   * @brief nav2_behavior_tree::TransformAvailableCondition的构造函数
   *        (A constructor for nav2_behavior_tree::TransformAvailableCondition)
   * @param condition_name 用于此节点的XML标签的名称 (Name for the XML tag for this node)
   * @param conf BT节点配置 (BT node configuration)
   */
  TransformAvailableCondition(
    const std::string & condition_name, const BT::NodeConfiguration & conf);

  // 删除默认构造函数 (Delete default constructor)
  TransformAvailableCondition() = delete;

  /**
   * @brief nav2_behavior_tree::TransformAvailableCondition的析构函数
   *        (A destructor for nav2_behavior_tree::TransformAvailableCondition)
   */
  ~TransformAvailableCondition();

  /**
   * @brief 主要需要由BT操作覆盖的方法 (The main override required by a BT action)
   * @return BT::NodeStatus 执行tick状态 (Status of tick execution)
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 创建BT端口列表 (Creates list of BT ports)
   * @return BT::PortsList 包含节点特定端口的列表 (List containing node-specific ports)
   */
  static BT::PortsList providedPorts()
  {
    return {// 孩子帧的输入端口 (Input port for child frame)
            BT::InputPort<std::string>("child", std::string(), "Child frame for transform"),
            // 父帧的输入端口 (Input port for parent frame)
            BT::InputPort<std::string>("parent", std::string(), "parent frame for transform")};
  }

private:
  rclcpp::Node::SharedPtr node_; // ROS2节点共享指针 (Shared pointer to a ROS2 node)
  std::shared_ptr<tf2_ros::Buffer>
    tf_; // tf2_ros缓冲区共享指针 (Shared pointer to a tf2_ros buffer)

  std::atomic<bool>
    was_found_; // 原子布尔值表示是否找到转换 (Atomic bool indicating if the transform was found)

  std::string child_frame_; // 孩子帧的字符串表示 (String representation of the child frame)
  std::string parent_frame_; // 父帧的字符串表示 (String representation of the parent frame)
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TRANSFORM_AVAILABLE_CONDITION_HPP_
