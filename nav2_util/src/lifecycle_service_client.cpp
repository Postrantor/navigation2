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

#include "nav2_util/lifecycle_service_client.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

using nav2_util::generate_internal_node;
using std::make_shared;
using std::string;
using std::chrono::seconds;
using namespace std::chrono_literals;

namespace nav2_util {

// clang-format off
/*
  其中包含了LifecycleServiceClient类的构造函数和三个成员函数：change_state、get_state。

  LifecycleServiceClient类的构造函数主要是通过传入生命周期节点名称或父节点来生成一个内部节点，并初始化change_state_和get_state_服务。
  在初始化服务之前，需要先等待服务启动，以保证服务可用。
  - change_state函数是用于请求生命周期节点进行状态转换的函数，其通过传入转换的状态和等待服务的超时时间，向生命周期节点请求状态转换，并返回服务调用是否成功。
  - get_state函数是用于获取生命周期节点的状态的函数，其通过传入等待服务的超时时间，向生命周期节点请求获取当前状态，并返回生命周期节点的状态。
*/

/**
 * @brief LifecycleServiceClient类的构造函数，用于创建一个LifecycleServiceClient对象
 * @param lifecycle_node_name 生命周期节点名称
 * @details 通过传入生命周期节点名称，生成一个内部节点，并初始化change_state_和get_state_服务
 */
LifecycleServiceClient::LifecycleServiceClient(const string& lifecycle_node_name)
    : node_(generate_internal_node(lifecycle_node_name + "_lifecycle_client")),
      change_state_(lifecycle_node_name + "/change_state", node_),
      get_state_(lifecycle_node_name + "/get_state", node_) {
  // Block until server is up
  rclcpp::Rate r(20);
  while (!get_state_.wait_for_service(2s)) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for service %s...", get_state_.getServiceName().c_str());
    r.sleep();
  }
}

/**
 * @brief LifecycleServiceClient类的构造函数，用于创建一个LifecycleServiceClient对象
 * @param lifecycle_node_name 生命周期节点名称
 * @param parent_node 父节点(在 plansys 中是 managed_node)
 * @details 通过传入生命周期节点名称和父节点，初始化change_state_和get_state_服务
 */
LifecycleServiceClient::LifecycleServiceClient(
    const string& lifecycle_node_name,  //
    rclcpp::Node::SharedPtr parent_node)
    : node_(parent_node),
      change_state_(lifecycle_node_name + "/change_state", node_),
      get_state_(lifecycle_node_name + "/get_state", node_) {
  // Block until server is up
  rclcpp::Rate r(20);
  while (!get_state_.wait_for_service(2s)) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for service %s...", get_state_.getServiceName().c_str());
    r.sleep();
  }
}

/**
 * @brief LifecycleServiceClient类的change_state函数，用于请求生命周期节点进行状态转换
 * @param transition 转换的状态
 * @param timeout 等待服务的超时时间
 * @return bool 返回服务调用是否成功
 * @details 该函数通过传入转换的状态和等待服务的超时时间，向生命周期节点请求状态转换，并返回服务调用是否成功
 */
bool LifecycleServiceClient::change_state(const uint8_t transition, const seconds timeout) {
  if (!change_state_.wait_for_service(timeout)) {
    throw std::runtime_error("change_state service is not available!");
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto response = change_state_.invoke(request, timeout);
  return response.get();
}

/**
 * @brief LifecycleServiceClient类的change_state函数，用于请求生命周期节点进行状态转换
 * @param transition 转换的状态
 * @return bool 返回服务调用是否成功
 * @details 该函数通过传入转换的状态，向生命周期节点请求状态转换，并返回服务调用是否成功
 */
bool LifecycleServiceClient::change_state(std::uint8_t transition) {
  if (!change_state_.wait_for_service(5s)) {
    throw std::runtime_error("change_state service is not available!");
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  auto response = std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();
  request->transition.id = transition;
  return change_state_.invoke(request, response);
}

/**
 * @brief LifecycleServiceClient类的get_state函数，用于获取生命周期节点的状态
 * @param timeout 等待服务的超时时间
 * @return uint8_t 返回生命周期节点的状态
 * @details
 *     该函数通过传入等待服务的超时时间，向生命周期节点请求获取当前状态，并返回生命周期节点的状态
 */
uint8_t LifecycleServiceClient::get_state(const seconds timeout) {
  if (!get_state_.wait_for_service(timeout)) {
    throw std::runtime_error("get_state service is not available!");
  }

  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto result = get_state_.invoke(request, timeout);
  return result->current_state.id;
}

}  // namespace nav2_util
