// Copyright (c) 2021 Joshua Wallace
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

#include "nav2_behavior_tree/plugins/condition/is_path_valid_condition.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace nav2_behavior_tree
{

/**
 * @brief 构造函数创建一个IsPathValidCondition节点 (Constructor for creating an IsPathValidCondition node)
 * @param condition_name 条件节点的名称 (Name of the condition node)
 * @param conf 节点配置 (Node configuration)
 */
IsPathValidCondition::IsPathValidCondition(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(
    condition_name, conf) // 继承自BT::ConditionNode类 (Inherits from BT::ConditionNode class)
{
  // 从黑板中获取共享指针类型的rclcpp::Node对象 (Get a shared pointer type rclcpp::Node object from the blackboard)
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // 使用节点创建"is_path_valid"服务客户端 (Create "is_path_valid" service client using the node)
  client_ = node_->create_client<nav2_msgs::srv::IsPathValid>("is_path_valid");

  // 从黑板中获取服务器超时时间 (Get server timeout duration from the blackboard)
  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");

  // 获取输入参数 "server_timeout" 的值 (Get the value of input parameter "server_timeout")
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
}

/**
 * @brief tick() 函数，用于执行条件检查 (tick() function for executing the condition check)
 * @return 返回条件检查结果 (Returns the result of the condition check)
 */
BT::NodeStatus IsPathValidCondition::tick()
{
  // 创建路径消息对象 (Create a path message object)
  nav_msgs::msg::Path path;

  // 获取输入参数 "path" 的值 (Get the value of input parameter "path")
  getInput("path", path);

  // 创建IsPathValid服务请求对象的共享指针 (Create a shared pointer for IsPathValid service request object)
  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();

  // 设置请求中的路径 (Set the path in the request)
  request->path = path;

  // 异步发送请求 (Send the request asynchronously)
  auto result = client_->async_send_request(request);

  // 等待结果，如果在超时时间内返回成功，则判断路径是否有效 (Wait for the result, if success is returned within the timeout, determine if the path is valid)
  if (
    rclcpp::spin_until_future_complete(node_, result, server_timeout_) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    if (result.get()->is_valid) {
      return BT::NodeStatus::SUCCESS; // 路径有效 (Path is valid)
    }
  }

  // 路径无效或未在超时时间内返回结果 (Path is invalid or result not returned within timeout)
  return BT::NodeStatus::FAILURE;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsPathValidCondition>("IsPathValid");
}
