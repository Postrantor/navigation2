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

#ifndef NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree {

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief Abstract class representing a service based BT node
 * @tparam ServiceT Type of service
 *
 * 表示基于服务的 BT 节点的抽象类
 * @tparam ServiceT 服务类型
 */
template <class ServiceT>
class BtServiceNode : public BT::ActionNodeBase {
public:
  /**
   * @brief A nav2_behavior_tree::BtServiceNode constructor
   * @param service_node_name BT node name
   * @param conf BT node configuration
   * @param service_name Optional service name this node creates a client for instead of from input
   * port
   *
   * nav2_behavior_tree::BtServiceNode 构造函数
   * @param service_node_name BT 节点名称
   * @param conf BT 节点配置
   * @param service_name 可选的服务名称，此节点为其创建客户端，而不是从输入端口
   */
  BtServiceNode(
      const std::string& service_node_name,
      const BT::NodeConfiguration& conf,
      const std::string& service_name = "")
      : BT::ActionNodeBase(service_node_name, conf),
        service_name_(service_name),
        service_node_name_(service_node_name) {
    // Get the shared pointer to the rclcpp::Node from the blackboard
    // 从黑板获取 rclcpp::Node 的共享指针
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");

    // Create a callback group and add it to the node's base interface
    // 创建回调组并将其添加到节点的基本接口中
    callback_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // Get the required items from the blackboard
    // 从黑板获取所需的项目
    bt_loop_duration_ =
        config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
    server_timeout_ =
        config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

    // Now that we have node_ to use, create the service client for this BT service
    // 现在我们有了可用的 node_，为此 BT 服务创建服务客户端
    getInput("service_name", service_name_);
    service_client_ =
        node_->create_client<ServiceT>(service_name_, rclcpp::SystemDefaultsQoS(), callback_group_);

    // Make a request for the service without parameter
    // 发起一个不带参数的服务请求
    request_ = std::make_shared<typename ServiceT::Request>();

    // Make sure the server is actually there before continuing
    // 在继续之前确保服务器确实存在
    RCLCPP_DEBUG(node_->get_logger(), "Waiting for \"%s\" service", service_name_.c_str());
    if (!service_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(
          node_->get_logger(), "\"%s\" service server not available after waiting for 1 s",
          service_node_name.c_str());
      throw std::runtime_error(
          std::string("Service server %s not available", service_node_name.c_str()));
    }

    // Log the successful initialization of the BtServiceNode
    // 记录 BtServiceNode 的成功初始化
    RCLCPP_DEBUG(
        node_->get_logger(), "\"%s\" BtServiceNode initialized", service_node_name_.c_str());
  }

  // Delete the default constructor
  // 删除默认构造函数
  BtServiceNode() = delete;

  // Declare a virtual destructor
  // 声明一个虚析构函数
  virtual ~BtServiceNode() {}

  /**
   * @brief 任何接受参数的 BtServiceNode 子类都必须提供一个 providedPorts 方法，并在其中调用
   * providedBasicPorts。
   * @brief Any subclass of BtServiceNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition 需要添加到 BT 端口列表中的附加端口
   * @param addition Additional ports to add to BT port list
   * @return 返回包含基本端口和节点特定端口的 BT::PortsList
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition) {
    // 定义基本端口列表，包括服务名称和服务器超时输入端口
    // Define basic ports list, including service name and server timeout input ports
    BT::PortsList basic = {
        BT::InputPort<std::string>("service_name", "please_set_service_name_in_BT_Node"),
        BT::InputPort<std::chrono::milliseconds>("server_timeout")};
    // 将 addition 中的端口插入到 basic 列表中
    // Insert ports from addition into the basic list
    basic.insert(addition.begin(), addition.end());

    // 返回包含基本端口和节点特定端口的 BT::PortsList
    // Return BT::PortsList containing basic ports along with node-specific ports
    return basic;
  }

  /**
   * @brief 创建 BT 端口列表
   * @brief Creates list of BT ports
   * @return 返回包含基本端口和节点特定端口的 BT::PortsList
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  /**
   * @brief 由 BT 服务需要的主要覆盖
   * @brief The main override required by a BT service
   * @return 返回 tick 执行的 BT::NodeStatus 状态
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override {
    // 如果请求尚未发送
    // If the request has not been sent yet
    if (!request_sent_) {
      // 重置是否发送请求的标志
      // Reset the flag to send the request or not,
      // 允许用户在 on_tick 中设置它
      // Allowing the user the option to set it in on_tick
      should_send_request_ = true;

      // 用户定义的回调，可能会修改 "should_send_request_"。
      // User-defined callback, may modify "should_send_request_".
      on_tick();

      // 如果不应该发送请求，则返回失败状态
      // If the request should not be sent, return failure status
      if (!should_send_request_) {
        return BT::NodeStatus::FAILURE;
      }

      // 异步发送请求并共享结果
      // Asynchronously send the request and share the result
      future_result_ = service_client_->async_send_request(request_).share();
      // 设置发送时间为当前节点时间
      // Set the sent time to the current node time
      sent_time_ = node_->now();
      // 设置请求已发送标志为 true
      // Set the request sent flag to true
      request_sent_ = true;
    }
    // 检查请求的执行结果
    // Check the execution result of the request
    return check_future();
  }

  /**
   * @brief 另一个（可选的）BT服务所需的覆写。The other (optional) override required by a BT
   * service.
   */
  void halt() override {
    // 将请求发送标志设置为false。Set the request_sent_ flag to false.
    request_sent_ = false;
    // 将节点状态设置为IDLE。Set the node status to IDLE.
    setStatus(BT::NodeStatus::IDLE);
  }

  /**
   * @brief 在tick上执行某些用户定义的操作。Function to perform some user-defined operation on tick
   * 如果需要，用信息填充服务请求。Fill in service request with information if necessary
   */
  virtual void on_tick() {}

  /**
   * @brief 在服务成功完成后执行某些用户定义的操作。Function to perform some user-defined operation
   * upon successful 完成服务后，可以将值放在黑板上。completion of the service. Could put a value on
   * the blackboard.
   * @param response 可以用来在BT节点中获取服务调用的结果。can be used to get the result of the
   * service call in the BT Node.
   * @return BT::NodeStatus 默认返回SUCCESS，用户可以覆盖为其他值。Returns SUCCESS by default, user
   * may override to return another value
   */
  virtual BT::NodeStatus on_completion(std::shared_ptr<typename ServiceT::Response> /*response*/) {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief 检查未来并决定BT的状态。Check the future and decide the status of BT
   * @return BT::NodeStatus 如果未来在超时之前完成，则为SUCCESS，否则为FAILURE。SUCCESS if future
   * complete before timeout, FAILURE otherwise
   */
  virtual BT::NodeStatus check_future() {
    // 计算已经过去的时间。Calculate the elapsed time.
    auto elapsed = (node_->now() - sent_time_).to_chrono<std::chrono::milliseconds>();
    // 计算剩余时间。Calculate the remaining time.
    auto remaining = server_timeout_ - elapsed;

    if (remaining > std::chrono::milliseconds(0)) {
      // 选择合适的超时时间。Choose an appropriate timeout.
      auto timeout = remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;

      rclcpp::FutureReturnCode rc;
      // 等待未来结果直到超时。Wait for the future result until timeout.
      rc = callback_group_executor_.spin_until_future_complete(future_result_, timeout);
      if (rc == rclcpp::FutureReturnCode::SUCCESS) {
        // 请求发送标志设置为false。Set the request_sent_ flag to false.
        request_sent_ = false;
        // 获取完成后的节点状态。Get the node status after completion.
        BT::NodeStatus status = on_completion(future_result_.get());
        return status;
      }

      if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        // 在等待尚未收到的结果超时后执行某些用户定义的操作。Perform some user-defined operation
        // after a timeout waiting for a result that hasn't been received yet.
        on_wait_for_result();
        // 更新已经过去的时间。Update the elapsed time.
        elapsed = (node_->now() - sent_time_).to_chrono<std::chrono::milliseconds>();
        if (elapsed < server_timeout_) {
          return BT::NodeStatus::RUNNING;
        }
      }
    }

    // 如果在执行服务调用时节点超时，则发出警告。Issue a warning if the node times out while
    // executing a service call.
    RCLCPP_WARN(
        node_->get_logger(), "Node timed out while executing service call to %s.",
        service_name_.c_str());
    // 将请求发送标志设置为false。Set the request_sent_ flag to false.
    request_sent_ = false;
    return BT::NodeStatus::FAILURE;
  }

  /**
   * @brief 在等待尚未收到的结果超时后执行某些用户定义的操作。Function to perform some user-defined
   * operation after a timeout waiting for a result that hasn't been received yet
   */
  virtual void on_wait_for_result() {}

protected:
  /**
   * @brief 增加黑板上恢复计数的函数（如果此节点包装了恢复功能）
   * @brief Function to increment recovery count on blackboard if this node wraps a recovery
   */
  void increment_recovery_count() {
    int recovery_count = 0;
    // NOLINT: 获取黑板上当前的恢复计数值
    // NOLINT: Get the current recovery count value from the blackboard
    config().blackboard->template get<int>("number_recoveries", recovery_count);

    recovery_count += 1;

    // NOLINT: 将更新后的恢复计数值设置回黑板
    // NOLINT: Set the updated recovery count value back to the blackboard
    config().blackboard->template set<int>("number_recoveries", recovery_count);
  }

  std::string service_name_, service_node_name_;
  typename std::shared_ptr<rclcpp::Client<ServiceT>> service_client_;
  std::shared_ptr<typename ServiceT::Request> request_;

  // 用于任何 ROS 操作的节点
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // 在 tick 循环中等待服务器响应时使用的超时值
  // The timeout value while to use in the tick loop while waiting for
  // a result from the server
  std::chrono::milliseconds server_timeout_;

  // BT 循环执行的超时值
  // The timeout value for BT loop execution
  std::chrono::milliseconds bt_loop_duration_;

  // 当发送新请求时跟踪服务器响应
  // To track the server response when a new request is sent
  std::shared_future<typename ServiceT::Response::SharedPtr> future_result_;
  bool request_sent_{false};
  rclcpp::Time sent_time_;

  // 可以在 on_tick 或 on_wait_for_result 中设置，以指示是否应发送请求。
  // Can be set in on_tick or on_wait_for_result to indicate if a request should be sent.
  bool should_send_request_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_
