// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_UTIL__SERVICE_CLIENT_HPP_
#define NAV2_UTIL__SERVICE_CLIENT_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace nav2_util {

/*
  其中定义了一个模板类ServiceClient，它是对ROS2服务进行调用的一个简单封装，支持invoke()和阻塞式调用。在构造函数中，首先创建了一个回调组，并将其添加到执行器中，然后创建了服务客户端。
  其中，service_name_表示服务名称，node_表示节点指针，callback_group_表示回调组指针，callback_group_executor_表示回调组执行器，client_表示服务客户端指针。
*/

/**
 * @class nav2_util::ServiceClient
 * @brief 一个简单的ROS2服务调用包装器，支持invoke()和阻塞式调用
 */
template <class ServiceT>
class ServiceClient {
public:
  /**
   * @brief 构造函数
   * @param service_name 要调用的服务名称
   * @param provided_node 创建服务客户端的节点
   */
  explicit ServiceClient(
      const std::string& service_name,  //
      const rclcpp::Node::SharedPtr& provided_node)
      : service_name_(service_name),    //
        node_(provided_node) {
    // 创建回调组
    callback_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    // 将回调组添加到执行器中
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    // 创建服务客户端
    client_ =
        node_->create_client<ServiceT>(service_name, rclcpp::SystemDefaultsQoS(), callback_group_);
  }

private:
  std::string service_name_;                               // 服务名称
  rclcpp::Node::SharedPtr node_;                           // 节点指针
  rclcpp::CallbackGroup::SharedPtr callback_group_;        // 回调组指针
  rclcpp::CallbackGroupExecutor callback_group_executor_;  // 回调组执行器
  typename rclcpp::Client<ServiceT>::SharedPtr client_;    // 服务客户端指针

  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  /*
    该函数是用于调用服务并等待响应的函数，其中包括等待服务出现、发送异步请求和等待异步请求完成等步骤。

    具体来说，该函数首先会等待服务出现，如果节点已经停止，则抛出异常；否则输出等待服务出现的信息。
    接着，它会发送异步请求，并等待异步请求完成，如果执行被中断或超时，则必须手动清除挂起的请求。
    最后，返回异步请求的结果。

    需要注意的是，该函数的参数包括请求对象和最大超时时间，默认为无限。
  */

  /**
   * @brief 调用服务并阻塞直到完成或超时
   * @param request 使用请求对象调用服务
   * @param timeout 最大超时时间，默认为无限
   * @return Response 请求的服务响应指针
   */
  typename ResponseType::SharedPtr invoke(
      typename RequestType::SharedPtr& request,
      const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1)) {
    // 等待服务出现，1s 查一次
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      // 如果节点已经停止，抛出异常
      if (!rclcpp::ok()) {
        throw std::runtime_error(
            service_name_ + " service client: interrupted while waiting for service");
      }
      // 输出等待服务出现的信息
      RCLCPP_INFO(
          node_->get_logger(), "%s service client: waiting for service to appear...",
          service_name_.c_str());
    }

    // 发送异步请求
    RCLCPP_DEBUG(
        node_->get_logger(), "%s service client: send async request", service_name_.c_str());
    auto future_result = client_->async_send_request(request);

    // 等待异步请求完成
    if (callback_group_executor_.spin_until_future_complete(future_result, timeout) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      // 如果执行被中断或超时，必须手动清除挂起的请求
      client_->remove_pending_request(future_result);
      throw std::runtime_error(service_name_ + " service client: async_send_request failed");
    }

    // 返回异步请求的结果
    return future_result.get();
  }

  /*
    代码段中的 invoke 函数是一个用于调用服务并等待响应的函数。在函数中，首先使用 wait_for_service
    函数等待服务可用，然后使用 async_send_request 函数发送异步请求，并使用
    spin_until_future_complete
    函数等待异步请求完成。如果执行被中断或超时，必须手动清除挂起的请求。最后，将响应存储在指向响应对象的指针中，并返回是否成功调用。

    wait_for_service 函数是一个阻塞函数，用于等待服务出现。如果服务在超时时间内出现，则返回
    true；否则返回 false。
  */

  /**
   * @brief 调用服务并阻塞直到完成
   * @param request 使用请求对象调用服务
   * @param Response 指向请求的服务响应的指针
   * @return bool 是否成功调用
   */
  bool invoke(
      typename RequestType::SharedPtr& request,  //
      typename ResponseType::SharedPtr& response) {
    // 等待服务出现
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
            service_name_ + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(
          node_->get_logger(), "%s service client: waiting for service to appear...",
          service_name_.c_str());
    }

    // 发送异步请求
    RCLCPP_DEBUG(
        node_->get_logger(), "%s service client: send async request", service_name_.c_str());
    auto future_result = client_->async_send_request(request);

    // 等待异步请求完成
    // 相比上面的函数，这里缺少了 timeout
    if (callback_group_executor_.spin_until_future_complete(future_result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      // 如果执行被中断或超时，必须手动清除挂起的请求
      client_->remove_pending_request(future_result);
      return false;
    }

    // 获取响应
    response = future_result.get();
    return response.get();
  }

  /**
   * @brief 阻塞直到服务可用或超时
   * @param timeout 最大等待时间，默认为无限
   * @return bool 如果服务可用，则返回true
   */
  bool wait_for_service(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max()) {
    return client_->wait_for_service(timeout);
  }

  /**
   * @brief 获取服务名称
   * @return string 服务名称
   */
  std::string getServiceName() { return service_name_; }

protected:
  std::string service_name_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__SERVICE_CLIENT_HPP_
