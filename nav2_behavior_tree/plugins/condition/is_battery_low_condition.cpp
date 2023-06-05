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

#include <string>

#include "nav2_behavior_tree/plugins/condition/is_battery_low_condition.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 初始化 IsBatteryLowCondition 类的构造函数
 *
 * @param condition_name 条件节点的名称（Name of the condition node）
 * @param conf 节点配置（Node configuration）
 */
IsBatteryLowCondition::IsBatteryLowCondition(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf), battery_topic_("/battery_status"), min_battery_(0.0),
  is_voltage_(false), is_battery_low_(false)
{
  // 获取输入参数 "min_battery" 的值（Get the value of input parameter "min_battery"）
  getInput("min_battery", min_battery_);
  // 获取输入参数 "battery_topic" 的值（Get the value of input parameter "battery_topic"）
  getInput("battery_topic", battery_topic_);
  // 获取输入参数 "is_voltage" 的值（Get the value of input parameter "is_voltage"）
  getInput("is_voltage", is_voltage_);
  // 从黑板中获取共享指针类型的 rclcpp::Node（Get rclcpp::Node::SharedPtr from the blackboard）
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  // 创建回调组（Create a callback group）
  callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  // 将回调组添加到回调组执行器中（Add the callback group to the callback group executor）
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  // 设置订阅选项，将回调组添加到选项中（Set subscription options and add the callback group to the options）
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  // 创建订阅（Create a subscription）
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&IsBatteryLowCondition::batteryCallback, this, std::placeholders::_1), sub_option);
}

/**
 * @brief 判断电池是否低电量的 tick 函数
 *
 * @return BT::NodeStatus 成功或失败状态（Success or failure status）
 */
BT::NodeStatus IsBatteryLowCondition::tick()
{
  // 执行回调组中的一些回调（Execute some callbacks in the callback group）
  callback_group_executor_.spin_some();
  // 如果电池低电量，返回成功状态（If the battery is low, return success status）
  if (is_battery_low_) {
    return BT::NodeStatus::SUCCESS;
  }
  // 否则返回失败状态（Otherwise, return failure status）
  return BT::NodeStatus::FAILURE;
}

/**
 * @brief 电池回调函数，处理电池信息
 *
 * @param msg 传感器消息类型的智能指针（SharedPtr of sensor_msgs::msg::BatteryState）
 */
void IsBatteryLowCondition::batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  // 根据 is_voltage_ 判断是比较电压还是百分比（Determine whether to compare voltage or percentage based on is_voltage_）
  if (is_voltage_) {
    // 如果电压小于等于 min_battery_，设置 is_battery_low_ 为 true（Set is_battery_low_ to true if the voltage is less than or equal to min_battery_）
    is_battery_low_ = msg->voltage <= min_battery_;
  } else {
    // 如果百分比小于等于 min_battery_，设置 is_battery_low_ 为 true（Set is_battery_low_ to true if the percentage is less than or equal to min_battery_）
    is_battery_low_ = msg->percentage <= min_battery_;
  }
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsBatteryLowCondition>("IsBatteryLow");
}
