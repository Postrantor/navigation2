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

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace nav2_behavior_tree {

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief 代表基于动作的 BT 节点的抽象类 (Abstract class representing an action based BT node)
 * @tparam ActionT 动作类型 (Type of action)
 */
template <class ActionT>
class BtActionNode : public BT::ActionNodeBase {
public:
  /**
   * @brief nav2_behavior_tree::BtActionNode 构造函数 (A nav2_behavior_tree::BtActionNode
   * constructor)
   * @param xml_tag_name 此节点的 XML 标签名 (Name for the XML tag for this node)
   * @param action_name 此节点为其创建客户端的动作名称 (Action name this node creates a client for)
   * @param conf BT 节点配置 (BT node configuration)
   */
  BtActionNode(
      const std::string& xml_tag_name,
      const std::string& action_name,
      const BT::NodeConfiguration& conf)
      : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name), should_send_goal_(true) {
    // 从黑板中获取节点 (Get the node from the blackboard)
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");

    // 创建互斥回调组 (Create a mutually exclusive callback group)
    callback_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // 从黑板中获取所需项目 (Get the required items from the blackboard)
    bt_loop_duration_ =
        config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
    server_timeout_ =
        config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

    // 初始化输入和输出消息 (Initialize the input and output messages)
    goal_ = typename ActionT::Goal();
    result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

    // 如果有输入"server_name"，则对动作名称进行重新映射 (Remap the action name if there's an input
    // "server_name")
    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      action_name_ = remapped_action_name;
    }
    createActionClient(action_name_);

    // 给派生类一个初始化的机会 (Give the derived class a chance to do any initialization)
    RCLCPP_DEBUG(node_->get_logger(), "\"%s\" BtActionNode initialized", xml_tag_name.c_str());
  }

  // 删除默认构造函数 (Delete the default constructor)
  BtActionNode() = delete;

  // 虚拟析构函数 (Virtual destructor)
  virtual ~BtActionNode() {}

  /**
   * @brief 创建一个动作客户端实例 (Create instance of an action client)
   * @param action_name 要为其创建客户端的动作名称 (Action name to create client for)
   */
  void createActionClient(const std::string& action_name) {
    // 现在我们有了要使用的 ROS 节点，为此 BT 动作创建动作客户端
    // (Now that we have the ROS node to use, create the action client for this BT action)
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name, callback_group_);

    // 在继续之前确保服务器实际上存在
    // (Make sure the server is actually there before continuing)
    RCLCPP_DEBUG(node_->get_logger(), "Waiting for \"%s\" action server", action_name.c_str());
    if (!action_client_->wait_for_action_server(1s)) {
      RCLCPP_ERROR(
          node_->get_logger(), "\"%s\" action server not available after waiting for 1 s",
          action_name.c_str());
      throw std::runtime_error(
          std::string("Action server ") + action_name + std::string(" not available"));
    }
  }

  /**
   * @brief 接受参数的 BtActionNode 的任何子类都必须提供一个 providedPorts 方法，并在其中调用
   * providedBasicPorts。 (Any subclass of BtActionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.)
   * @param addition 要添加到 BT 端口列表中的附加端口 (Additional ports to add to BT port list)
   * @return BT::PortsList 包含基本端口以及特定于节点的端口 (Containing basic ports along with
   * node-specific ports)
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition) {
    BT::PortsList basic = {
        BT::InputPort<std::string>("server_name", "Action server name"),
        BT::InputPort<std::chrono::milliseconds>("server_timeout")};
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief 创建 BT 端口列表 (Creates list of BT ports)
   * @return BT::PortsList 包含基本端口以及特定于节点的端口 (Containing basic ports along with
   * node-specific ports)
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  // 派生类可以重写以下任何方法，以便在处理操作时挂钩：
  // on_tick、on_wait_for_result 和 on_success
  // (Derived classes can override any of the following methods to hook into the
  // processing for the action: on_tick, on_wait_for_result, and on_success)

  /**
   * @brief 在 tick 上执行某些用户定义的操作
   * 可以进行动态检查，例如获取黑板上值的更新
   * (Function to perform some user-defined operation on tick
   * Could do dynamic checks, such as getting updates to values on the blackboard)
   */
  virtual void on_tick() {}

  /**
   * @brief Function to perform some user-defined operation after a timeout
   * waiting for a result that hasn't been received yet. Also provides access to
   * the latest feedback message from the action server. Feedback will be nullptr
   * in subsequent calls to this function if no new feedback is received while waiting for a result.
   * @param feedback shared_ptr to latest feedback message, nullptr if no new feedback was received
   *
   * @brief
   * 在等待尚未收到的结果超时后执行某些用户定义的操作的函数。还可以访问来自操作服务器的最新反馈消息。
   * 如果在等待结果时没有收到新的反馈，那么在对此函数的后续调用中反馈将为 nullptr。
   * @param feedback 指向最新反馈消息的 shared_ptr，如果没有收到新反馈，则为 nullptr
   */
  virtual void on_wait_for_result(std::shared_ptr<const typename ActionT::Feedback> /*feedback*/) {}

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the action. Could put a value on the blackboard.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   *
   * @brief 在操作成功完成时执行某些用户定义的操作的函数。可以在黑板上放置一个值。
   * @return BT::NodeStatus 默认返回 SUCCESS，用户可以覆盖返回另一个值
   */
  virtual BT::NodeStatus on_success() { return BT::NodeStatus::SUCCESS; }

  /**
   * @brief Function to perform some user-defined operation when the action is aborted.
   * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
   *
   * @brief 当操作被中止时执行某些用户定义的操作的函数。
   * @return BT::NodeStatus 默认返回 FAILURE，用户可以覆盖返回另一个值
   */
  virtual BT::NodeStatus on_aborted() { return BT::NodeStatus::FAILURE; }

  /**
   * @brief Function to perform some user-defined operation when the action is cancelled.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   *
   * @brief 当操作被取消时执行某些用户定义的操作的函数。
   * @return BT::NodeStatus 默认返回 SUCCESS，用户可以覆盖返回另一个值
   */
  virtual BT::NodeStatus on_cancelled() { return BT::NodeStatus::SUCCESS; }

  /**
   * @brief 主要的 BT action 需要覆盖的函数 (The main override required by a BT action)
   * @return BT::NodeStatus tick 执行的状态 (Status of tick execution)
   */
  BT::NodeStatus tick() override {
    // 只在 Action 开始时执行的第一步 (First step to be done only at the beginning of the Action)
    if (status() == BT::NodeStatus::IDLE) {
      // 设置状态为 RUNNING，以通知 BT Loggers（如果有的话）(Setting the status to RUNNING to notify
      // the BT Loggers (if any))
      setStatus(BT::NodeStatus::RUNNING);

      // 重置是否发送目标的标志，允许用户在 on_tick 中设置它 (Reset the flag to send the goal or
      // not, allowing the user the option to set it in on_tick)
      should_send_goal_ = true;

      // 用户定义的回调，可能会修改 "should_send_goal_" (User defined callback, may modify
      // "should_send_goal_".)
      on_tick();

      if (!should_send_goal_) {
        return BT::NodeStatus::FAILURE;
      }
      send_new_goal();
    }

    try {
      // 如果发送了新目标且操作服务器尚未响应 (If new goal was sent and action server has not yet
      // responded) 检查未来目标句柄 (Check the future goal handle)
      if (future_goal_handle_) {
        auto elapsed = (node_->now() - time_goal_sent_).to_chrono<std::chrono::milliseconds>();
        if (!is_future_goal_handle_complete(elapsed)) {
          // 在超时发生之前仍有一些时间返回 RUNNING (Return RUNNING if there is still some time
          // before timeout happens)
          if (elapsed < server_timeout_) {
            return BT::NodeStatus::RUNNING;
          }
          // 如果服务器花费的时间超过指定的超时值，则返回 FAILURE (Return FAILURE if the server has
          // taken more time than the specified timeout value)
          RCLCPP_WARN(
              node_->get_logger(),
              "Timed out while waiting for action server to acknowledge goal request for %s",
              action_name_.c_str());
          future_goal_handle_.reset();
          return BT::NodeStatus::FAILURE;
        }
      }

      // 以下代码对应于 "RUNNING" 循环 (The following code corresponds to the "RUNNING" loop)
      if (rclcpp::ok() && !goal_result_available_) {
        // 用户定义的回调。可能会修改 "goal_updated_" 的值 (User defined callback. May modify the
        // value of "goal_updated_")
        on_wait_for_result(feedback_);

        // 重置反馈以避免过时的信息 (Reset feedback to avoid stale information)
        feedback_.reset();

        auto goal_status = goal_handle_->get_status();
        if (goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
                              goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED)) {
          goal_updated_ = false;
          send_new_goal();
          auto elapsed = (node_->now() - time_goal_sent_).to_chrono<std::chrono::milliseconds>();
          if (!is_future_goal_handle_complete(elapsed)) {
            if (elapsed < server_timeout_) {
              return BT::NodeStatus::RUNNING;
            }
            RCLCPP_WARN(
                node_->get_logger(),
                "Timed out while waiting for action server to acknowledge goal request for %s",
                action_name_.c_str());
            future_goal_handle_.reset();
            return BT::NodeStatus::FAILURE;
          }
        }

        callback_group_executor_.spin_some();

        // 检查是否在调用 spin_some() 之后，我们最终收到了结果 (Check if, after invoking
        // spin_some(), we finally received the result)
        if (!goal_result_available_) {
          // 让出此 Action，返回 RUNNING (Yield this Action, returning RUNNING)
          return BT::NodeStatus::RUNNING;
        }
      }
    } catch (const std::runtime_error& e) {
      if (e.what() == std::string("send_goal failed") ||
          e.what() == std::string("Goal was rejected by the action server")) {
        // 与操作相关的故障，不应使树失败，但节点可能会失败 (Action related failure that should not
        // fail the tree, but the node)
        return BT::NodeStatus::FAILURE;
      } else {
        // 内部异常传播到树 (Internal exception to propagate to the tree)
        throw e;
      }
    }

    BT::NodeStatus status;
    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        status = on_success();
        break;

      case rclcpp_action::ResultCode::ABORTED:
        status = on_aborted();
        break;

      case rclcpp_action::ResultCode::CANCELED:
        status = on_cancelled();
        break;

      default:
        throw std::logic_error("BtActionNode::Tick: invalid status value");
    }

    goal_handle_.reset();
    return status;
  }

  /**
   * @brief 另一个（可选的）BT动作所需的重写。在这种情况下，我们确保在 ROS2 操作仍在运行时取消它。
   * @brief The other (optional) override required by a BT action. In this case, we
   * make sure to cancel the ROS2 action if it is still running.
   */
  void halt() override {
    // 如果目标需要被取消，则执行取消操作
    // If the goal should be canceled, perform the cancel operation
    if (should_cancel_goal()) {
      // 异步取消目标
      // Asynchronously cancel the goal
      auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
      // 等待取消操作完成或超时，并检查结果是否成功
      // Wait for the cancel operation to complete or time out, and check if the result is
      // successful
      if (callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        // 如果取消失败，记录错误日志
        // Log an error if the cancellation fails
        RCLCPP_ERROR(
            node_->get_logger(), "Failed to cancel action server for %s", action_name_.c_str());
      }
    }

    // 设置节点状态为 IDLE
    // Set the node status to IDLE
    setStatus(BT::NodeStatus::IDLE);
  }

protected:
  /**
   * @brief 检查当前目标是否应该被取消的函数
   * @brief Function to check if current goal should be cancelled
   * @return bool 如果当前目标应该被取消，则返回 true，否则返回 false
   * @return bool True if current goal should be cancelled, false otherwise
   */
  bool should_cancel_goal() {
    // 如果节点当前正在运行，关闭它
    // Shut the node down if it is currently running
    if (status() != BT::NodeStatus::RUNNING) {
      return false;
    }

    // 如果目标句柄无效，则无需取消目标
    // No need to cancel the goal if goal handle is invalid
    if (!goal_handle_) {
      return false;
    }

    // 执行部分回调组操作
    // Execute some callback group operations
    callback_group_executor_.spin_some();
    // 获取目标的状态
    // Get the status of the goal
    auto status = goal_handle_->get_status();

    // 检查目标是否仍在执行
    // Check if the goal is still executing
    return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
           status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }

  /**
   * @brief 发送新目标给动作服务器 (Function to send new goal to action server)
   *
   * @param[in] 无 (None)
   * @param[out] 无 (None)
   */
  void send_new_goal() {
    // 设置目标结果是否可用的标志为 false (Set the flag for whether the goal result is available to
    // false)
    goal_result_available_ = false;

    // 创建发送目标选项对象 (Create a SendGoalOptions object)
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();

    // 设置结果回调，当收到目标结果时调用 (Set the result callback to be called when the goal result
    // is received)
    send_goal_options.result_callback =
        [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult& result) {
          // 如果 future_goal_handle_ 存在，则说明还没有收到目标响应 (If future_goal_handle_ exists,
          // it means that the goal response has not been received yet)
          if (future_goal_handle_) {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "Goal result for %s available, but it hasn't received the goal response yet. "
                "It's probably a goal result for the last goal request",
                action_name_.c_str());
            return;
          }

          // TODO(#1652): a work around until rcl_action interface is updated
          // 如果目标 ID 不匹配，则忽略此结果，否则必须处理结果（包括中止）(If the goal IDs do not
          // match, ignore this result; otherwise, it must be processed (including aborted))
          if (this->goal_handle_->get_goal_id() == result.goal_id) {
            goal_result_available_ = true;
            result_ = result;
          }
        };

    // 设置反馈回调，当收到目标进度时调用 (Set the feedback callback to be called when the goal
    // progress is received)
    send_goal_options.feedback_callback =
        [this](
            typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
            const std::shared_ptr<const typename ActionT::Feedback> feedback) {
          feedback_ = feedback;
        };

    // 异步发送目标，并将返回的 future 对象存储在 future_goal_handle_ 中 (Asynchronously send the
    // goal and store the returned future object in future_goal_handle_)
    future_goal_handle_ = std::make_shared<
        std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>(
        action_client_->async_send_goal(goal_, send_goal_options));

    // 记录发送目标的时间 (Record the time when the goal is sent)
    time_goal_sent_ = node_->now();
  }

  /**
   * @brief 检查动作服务器是否确认了新的目标 (Function to check if the action server acknowledged a
   * new goal)
   * @param elapsed
   * 自上一个目标发送以来的持续时间，未来目标句柄尚未完成。在等待未来完成后，此值将增加超时值。
   * (Duration since the last goal was sent and future goal handle has not completed.
   * After waiting for the future to complete, this value is incremented with the timeout value.)
   * @return boolean 如果 future_goal_handle_ 返回 SUCCESS，则为 True；否则为 False。
   * (True if future_goal_handle_ returns SUCCESS, False otherwise)
   */
  bool is_future_goal_handle_complete(std::chrono::milliseconds& elapsed) {
    // 计算剩余时间 (Calculate the remaining time)
    auto remaining = server_timeout_ - elapsed;

    // 如果服务器已经超时，无需休眠 (If the server has already timed out, there's no need to sleep)
    if (remaining <= std::chrono::milliseconds(0)) {
      future_goal_handle_.reset();
      return false;
    }

    // 计算超时时间 (Calculate the timeout duration)
    auto timeout = remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;
    // 等待 future_goal_handle_ 完成或超时 (Wait for future_goal_handle_ to complete or timeout)
    auto result =
        callback_group_executor_.spin_until_future_complete(*future_goal_handle_, timeout);
    // 更新已用时间 (Update the elapsed time)
    elapsed += timeout;

    // 如果结果被中断，重置 future_goal_handle_ 并抛出异常 (If the result was interrupted, reset
    // future_goal_handle_ and throw an exception)
    if (result == rclcpp::FutureReturnCode::INTERRUPTED) {
      future_goal_handle_.reset();
      throw std::runtime_error("send_goal failed");
    }

    // 如果结果成功，获取 goal_handle_ 并重置 future_goal_handle_ (If the result was successful, get
    // the goal_handle_ and reset future_goal_handle_)
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      goal_handle_ = future_goal_handle_->get();
      future_goal_handle_.reset();
      // 如果 goal_handle_ 为空，抛出异常 (If goal_handle_ is empty, throw an exception)
      if (!goal_handle_) {
        throw std::runtime_error("Goal was rejected by the action server");
      }
      return true;
    }

    // 其他情况返回 false (In other cases, return false)
    return false;
  }

  /**
   * @brief 用于在黑板上增加恢复次数的函数，如果该节点包装了一个恢复操作
   * @brief Function to increment recovery count on blackboard if this node wraps a recovery
   */
  void increment_recovery_count() {
    // 定义一个整型变量，用于存储恢复次数
    // Define an integer variable to store the recovery count
    int recovery_count = 0;

    // 从黑板中获取当前的恢复次数，并将其存储在recovery_count中
    // Get the current recovery count from the blackboard and store it in recovery_count
    config().blackboard->template get<int>("number_recoveries", recovery_count);  // NOLINT

    // 将恢复次数加1
    // Increment the recovery count by 1
    recovery_count += 1;

    // 将更新后的恢复次数设置回黑板
    // Set the updated recovery count back to the blackboard
    config().blackboard->template set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  // 定义一个字符串变量，用于存储动作名称
  // Define a string variable to store the action name
  std::string action_name_;

  // 定义一个共享指针，用于指向rclcpp_action::Client<ActionT>类型的对象
  // Define a shared pointer pointing to an object of type rclcpp_action::Client<ActionT>
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

  // 所有ROS2动作都有一个目标和一个结果
  // All ROS2 actions have a goal and a result
  typename ActionT::Goal goal_;
  bool goal_updated_{false};
  bool goal_result_available_{false};
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

  // 用于处理来自动作服务器的反馈
  // To handle feedback from action server
  std::shared_ptr<const typename ActionT::Feedback> feedback_;

  // 将用于任何ROS操作的节点
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // 在发送或取消新的动作目标时等待服务器响应的超时值
  // The timeout value while waiting for response from a server when a new action goal is sent or
  // canceled
  std::chrono::milliseconds server_timeout_;

  // BT循环执行的超时值
  // The timeout value for BT loop execution
  std::chrono::milliseconds bt_loop_duration_;

  // 跟踪在发送新目标时动作服务器的确认
  // To track the action server acknowledgement when a new goal is sent
  std::shared_ptr<std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>
      future_goal_handle_;
  rclcpp::Time time_goal_sent_;

  // 可以在on_tick或on_wait_for_result中设置，以指示是否应发送目标。
  // Can be set in on_tick or on_wait_for_result to indicate if a goal should be sent.
  bool should_send_goal_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
