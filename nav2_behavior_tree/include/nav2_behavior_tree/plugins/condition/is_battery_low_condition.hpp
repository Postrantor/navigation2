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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 一个BT::ConditionNode，监听电池主题，当电池电量低时返回SUCCESS，否则返回FAILURE。
 *        (A BT::ConditionNode that listens to a battery topic and
 *         returns SUCCESS when battery is low and FAILURE otherwise)
 */
class IsBatteryLowCondition : public BT::ConditionNode
{
public:
  /**
   * @brief nav2_behavior_tree::IsBatteryLowCondition 的构造函数
   *        (A constructor for nav2_behavior_tree::IsBatteryLowCondition)
   * @param condition_name 此节点的 XML 标签名称 (Name for the XML tag for this node)
   * @param conf BT 节点配置 (BT node configuration)
   */
  IsBatteryLowCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

  // 删除默认构造函数 (Delete default constructor)
  IsBatteryLowCondition() = delete;

  /**
   * @brief 需要由 BT 动作重写的主要方法 (The main override required by a BT action)
   * @return BT::NodeStatus 执行 tick 的状态 (Status of tick execution)
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 创建 BT 端口列表 (Creates list of BT ports)
   * @return BT::PortsList 包含特定于节点的端口 (Containing node-specific ports)
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>(
        "min_battery", "最低电池百分比/电压 (Minimum battery percentage/voltage)"),
      BT::InputPort<std::string>(
        "battery_topic", std::string("/battery_status"), "电池主题 (Battery topic)"),
      BT::InputPort<bool>(
        "is_voltage", false,
        "如果为 true，则使用电压检查低电池 (If true voltage will be used to check for low "
        "battery)"),
    };
  }

private:
  /**
   * @brief 电池主题的回调函数 (Callback function for battery topic)
   * @param msg 指向 sensor_msgs::msg::BatteryState 消息的共享指针
   *            (Shared pointer to sensor_msgs::msg::BatteryState message)
   */
  void batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;                    // ROS2 节点 (ROS2 node)
  rclcpp::CallbackGroup::SharedPtr callback_group_; // 回调组 (Callback group)
  rclcpp::executors::SingleThreadedExecutor
    callback_group_executor_; // 单线程执行器 (Single-threaded executor)
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr
    battery_sub_;             // 电池订阅 (Battery subscription)
  std::string battery_topic_; // 电池主题 (Battery topic)
  double min_battery_;        // 最低电池百分比/电压 (Minimum battery percentage/voltage)
  bool is_voltage_; // 是否使用电压检查低电池 (Whether to use voltage to check for low battery)
  bool is_battery_low_; // 电池是否低 (Whether the battery is low)
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
