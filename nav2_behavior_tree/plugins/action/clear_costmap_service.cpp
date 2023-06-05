// Copyright (c) 2019 Samsung Research America
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

#include "nav2_behavior_tree/plugins/action/clear_costmap_service.hpp"

#include <memory>
#include <string>

namespace nav2_behavior_tree {

// ClearEntireCostmapService 构造函数，用于清空整个代价地图
ClearEntireCostmapService::ClearEntireCostmapService(
    const std::string& service_node_name, const BT::NodeConfiguration& conf)
    : BtServiceNode<nav2_msgs::srv::ClearEntireCostmap>(service_node_name, conf) {}

// on_tick 函数，每次 tick 时调用，用于增加恢复计数器的值
void ClearEntireCostmapService::on_tick() { increment_recovery_count(); }

// ClearCostmapExceptRegionService 构造函数，用于清空除指定区域外的代价地图
ClearCostmapExceptRegionService::ClearCostmapExceptRegionService(
    const std::string& service_node_name, const BT::NodeConfiguration& conf)
    : BtServiceNode<nav2_msgs::srv::ClearCostmapExceptRegion>(service_node_name, conf) {}

// on_tick 函数，每次 tick 时调用，用于获取输入参数 reset_distance 的值并增加恢复计数器的值
void ClearCostmapExceptRegionService::on_tick() {
  getInput("reset_distance", request_->reset_distance);
  increment_recovery_count();
}

// ClearCostmapAroundRobotService 构造函数，用于清空机器人周围的代价地图
ClearCostmapAroundRobotService::ClearCostmapAroundRobotService(
    const std::string& service_node_name, const BT::NodeConfiguration& conf)
    : BtServiceNode<nav2_msgs::srv::ClearCostmapAroundRobot>(service_node_name, conf) {}

// on_tick 函数，每次 tick 时调用，用于获取输入参数 reset_distance 的值并增加恢复计数器的值
void ClearCostmapAroundRobotService::on_tick() {
  getInput("reset_distance", request_->reset_distance);
  increment_recovery_count();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"

// 注册三个节点类型，分别是清空整个代价地图、清空除指定区域外的代价地图和清空机器人周围的代价地图
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::ClearEntireCostmapService>("ClearEntireCostmap");
  factory.registerNodeType<nav2_behavior_tree::ClearCostmapExceptRegionService>(
      "ClearCostmapExceptRegion");
  factory.registerNodeType<nav2_behavior_tree::ClearCostmapAroundRobotService>(
      "ClearCostmapAroundRobot");
}
