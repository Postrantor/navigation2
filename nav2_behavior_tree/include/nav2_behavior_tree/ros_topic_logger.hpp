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

#ifndef NAV2_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_
#define NAV2_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_

#include <memory>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "nav2_msgs/msg/behavior_tree_status_change.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer_interface.h"

namespace nav2_behavior_tree {

/**
 * @brief 一个在行为树 (BT) 状态改变时发布 BT 日志的类 (A class to publish BT logs on BT status
 * change)
 */
class RosTopicLogger : public BT::StatusChangeLogger {
public:
  /**
   * @brief nav2_behavior_tree::RosTopicLogger 的构造函数 (A constructor for
   * nav2_behavior_tree::RosTopicLogger)
   * @param ros_node 指向父 rclcpp::Node 的弱指针 (Weak pointer to parent rclcpp::Node)
   * @param tree 要监控的 BT (BT to monitor)
   */
  RosTopicLogger(const rclcpp::Node::WeakPtr& ros_node, const BT::Tree& tree)
      : StatusChangeLogger(tree.rootNode()) {
    // 锁定 ros_node 并获取节点、时钟和日志器 (Lock ros_node and get the node, clock, and logger)
    auto node = ros_node.lock();
    clock_ = node->get_clock();
    logger_ = node->get_logger();

    // 创建发布器用于发布行为树日志 (Create publisher for publishing behavior tree logs)
    log_pub_ = node->create_publisher<nav2_msgs::msg::BehaviorTreeLog>(
        "behavior_tree_log", rclcpp::QoS(10));
  }

  /**
   * @brief 当 BT 状态发生变化时调用的回调函数 (Callback function which is called each time BT
   * changes status)
   * @param timestamp BT 状态改变的时间戳 (Timestamp of BT status change)
   * @param node 改变状态的节点 (Node that changed status)
   * @param prev_status 节点的前一个状态 (Previous status of the node)
   * @param status 节点的当前状态 (Current status of the node)
   */
  void callback(
      BT::Duration timestamp,
      const BT::TreeNode& node,
      BT::NodeStatus prev_status,
      BT::NodeStatus status) override {
    // 创建一个行为树状态改变事件对象 (Create a behavior tree status change event object)
    nav2_msgs::msg::BehaviorTreeStatusChange event;

    // 将 BT 时间戳从纪元以来的持续时间转换为 time_point，然后再转换为消息 (Convert BT timestamps
    // from duration since the epoch to a time_point, then to a msg)
    event.timestamp = tf2_ros::toMsg(tf2::TimePoint(timestamp));
    event.node_name = node.name();
    event.previous_status = toStr(prev_status, false);
    event.current_status = toStr(status, false);

    // 将事件添加到事件日志中 (Add the event to the event log)
    event_log_.push_back(std::move(event));

    // 记录日志 (Log the event)
    RCLCPP_DEBUG(
        logger_, "[%.3f]: %25s %s -> %s", std::chrono::duration<double>(timestamp).count(),
        node.name().c_str(), toStr(prev_status, true).c_str(), toStr(status, true).c_str());
  }

  /**
   * @brief 清除日志缓冲区（如果有）(Clear log buffer if any)
   */
  void flush() override {
    if (!event_log_.empty()) {
      // 创建一个行为树日志消息 (Create a behavior tree log message)
      auto log_msg = std::make_unique<nav2_msgs::msg::BehaviorTreeLog>();
      log_msg->timestamp = clock_->now();
      log_msg->event_log = event_log_;

      // 发布日志消息 (Publish the log message)
      log_pub_->publish(std::move(log_msg));

      // 清除事件日志 (Clear the event log)
      event_log_.clear();
    }
  }

protected:
  rclcpp::Clock::SharedPtr clock_;                             // 时钟对象 (Clock object)
  rclcpp::Logger logger_{rclcpp::get_logger("bt_navigator")};  // 日志器对象 (Logger object)
  rclcpp::Publisher<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr
      log_pub_;  // 行为树日志发布器 (Behavior tree log publisher)
  std::vector<nav2_msgs::msg::BehaviorTreeStatusChange> event_log_;  // 事件日志 (Event log)
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_
