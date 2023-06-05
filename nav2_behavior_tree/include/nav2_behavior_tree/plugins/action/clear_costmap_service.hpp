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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_COSTMAP_SERVICE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_COSTMAP_SERVICE_HPP_

#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include "nav2_msgs/srv/clear_costmap_except_region.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 一个包装 nav2_msgs::srv::ClearEntireCostmap 的 nav2_behavior_tree::BtServiceNode 类 (A nav2_behavior_tree::BtServiceNode class that wraps nav2_msgs::srv::ClearEntireCostmap)
 */
class ClearEntireCostmapService : public BtServiceNode<nav2_msgs::srv::ClearEntireCostmap>
{
public:
  /**
   * @brief nav2_behavior_tree::ClearEntireCostmapService 的构造函数 (A constructor for nav2_behavior_tree::ClearEntireCostmapService)
   * @param service_node_name 此节点为其创建客户端的服务名称 (Service name this node creates a client for)
   * @param conf BT 节点配置 (BT node configuration)
   */
  ClearEntireCostmapService(
    const std::string & service_node_name, const BT::NodeConfiguration & conf);

  /**
   * @brief BT 服务所需的主要覆盖 (The main override required by a BT service)
   * @return BT::NodeStatus 执行 tick 的状态 (Status of tick execution)
   */
  void on_tick() override;
};

/**
 * @brief 一个包装 nav2_msgs::srv::ClearCostmapExceptRegion 的 nav2_behavior_tree::BtServiceNode 类 (A nav2_behavior_tree::BtServiceNode class that wraps nav2_msgs::srv::ClearCostmapExceptRegion)
 */
class ClearCostmapExceptRegionService
: public BtServiceNode<nav2_msgs::srv::ClearCostmapExceptRegion>
{
public:
  /**
   * @brief nav2_behavior_tree::ClearCostmapExceptRegionService 的构造函数 (A constructor for nav2_behavior_tree::ClearCostmapExceptRegionService)
   * @param service_node_name 此节点为其创建客户端的服务名称 (Service name this node creates a client for)
   * @param conf BT 节点配置 (BT node configuration)
   */
  ClearCostmapExceptRegionService(
    const std::string & service_node_name, const BT::NodeConfiguration & conf);

  /**
   * @brief BT 服务所需的主要覆盖 (The main override required by a BT service)
   * @return BT::NodeStatus 执行 tick 的状态 (Status of tick execution)
   */
  void on_tick() override;

  /**
   * @brief 创建 BT 端口列表 (Creates list of BT ports)
   * @return BT::PortsList 包含基本端口和节点特定端口的列表 (Containing basic ports along with node-specific ports)
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<double>(
      "reset_distance", 1,
      "清除机器人距离以上的障碍物的距离 (Distance from the robot above which obstacles are "
      "cleared)")});
  }
};

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that
 * wraps nav2_msgs::srv::ClearCostmapAroundRobot
 *
 * 这是一个nav2_behavior_tree::BtServiceNode类，封装了nav2_msgs::srv::ClearCostmapAroundRobot服务
 */
class ClearCostmapAroundRobotService : public BtServiceNode<nav2_msgs::srv::ClearCostmapAroundRobot>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::ClearCostmapAroundRobotService
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   *
   * nav2_behavior_tree::ClearCostmapAroundRobotService的构造函数
   * @param service_node_name 该节点为其创建客户端的服务名称
   * @param conf 行为树节点配置
   */
  ClearCostmapAroundRobotService(
    const std::string & service_node_name, const BT::NodeConfiguration & conf);

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   *
   * 行为树服务所需的主要覆盖
   * @return BT::NodeStatus tick执行状态
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   *
   * 创建行为树端口列表
   * @return BT::PortsList 包含基本端口以及节点特定端口
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<double>(
      "reset_distance", 1, "Distance from the robot under which obstacles are cleared")});
    // 返回提供的基本端口，包括输入端口"reset_distance"，其默认值为1，表示清除机器人周围障碍物的距离
  }
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_COSTMAP_SERVICE_HPP_
