// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_

#include <exception>
#include <fstream>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace nav2_behavior_tree {

/**
 * @brief BtActionServer 构造函数
 *
 * @tparam ActionT 动作类型
 * @param parent 生命周期节点的弱指针
 * @param action_name 动作名字
 * @param plugin_lib_names 插件库名字列表
 * @param default_bt_xml_filename 默认行为树 XML 文件名
 * @param on_goal_received_callback 收到目标回调函数
 * @param on_loop_callback 循环回调函数
 * @param on_preempt_callback 抢占回调函数
 * @param on_completion_callback 完成回调函数
 */
template <class ActionT>
BtActionServer<ActionT>::BtActionServer(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,  ///< [in] 生命周期节点的弱指针
    const std::string& action_name,                          ///< [in] 动作名字
    const std::vector<std::string>& plugin_lib_names,        ///< [in] 插件库名字列表
    const std::string& default_bt_xml_filename,              ///< [in] 默认行为树 XML 文件名
    OnGoalReceivedCallback on_goal_received_callback,        ///< [in] 收到目标回调函数
    OnLoopCallback on_loop_callback,                         ///< [in] 循环回调函数
    OnPreemptCallback on_preempt_callback,                   ///< [in] 抢占回调函数
    OnCompletionCallback on_completion_callback)             ///< [in] 完成回调函数
    : action_name_(action_name),
      default_bt_xml_filename_(default_bt_xml_filename),
      plugin_lib_names_(plugin_lib_names),
      node_(parent),
      on_goal_received_callback_(on_goal_received_callback),
      on_loop_callback_(on_loop_callback),
      on_preempt_callback_(on_preempt_callback),
      on_completion_callback_(on_completion_callback) {
  // 锁定节点并获取日志记录器和时钟
  auto node = node_.lock();      ///< 获得生命周期节点的共享指针
  logger_ = node->get_logger();  ///< 获取日志记录器
  clock_ = node->get_clock();    ///< 获取时钟

  // 声明此节点的参数
  if (!node->has_parameter("bt_loop_duration")) {
    node->declare_parameter("bt_loop_duration", 10);  ///< 声明 bt_loop_duration 参数，默认值为 10
  }
  if (!node->has_parameter("default_server_timeout")) {
    node->declare_parameter(
        "default_server_timeout", 20);  ///< 声明 default_server_timeout 参数，默认值为 20
  }

  std::vector<std::string> error_code_names = {
      "follow_path_error_code", "compute_path_error_code"};  ///< 定义错误代码名称列表

  /*!
   * @brief 检查节点是否已有 "error_code_names" 参数，若没有则使用默认值并添加警告日志。
   *        Check if the node has the parameter "error_code_names", if not, use default values and
   * log a warning.
   */
  if (!node->has_parameter("error_code_names")) {
    // 初始化一个空字符串用于存储错误代码名称
    // Initialize an empty string to store error code names
    std::string error_codes_str;

    // 遍历错误代码名称列表
    // Iterate through the error code names list
    for (const auto& error_code : error_code_names) {
      // 将当前错误代码名称添加到字符串中，并在每个名称后添加换行符
      // Add the current error code name to the string, appending a newline character after each
      // name
      error_codes_str += error_code + "\n";
    }

    // 输出警告日志，提示用户 "error_code_names" 参数未设置，将使用默认值
    // Log a warning message indicating that the "error_code_names" parameter was not set and
    // default values will be used
    RCLCPP_WARN_STREAM(
        logger_, "Error_code parameters were not set. Using default values of: "
                     << error_codes_str
                     << "Make sure these match your BT and there are not other sources of error "
                        "codes you want "
                        "reported to your application");

    // 在节点中声明 "error_code_names" 参数并设置为默认值
    // Declare the "error_code_names" parameter in the node and set it to the default value
    node->declare_parameter("error_code_names", error_code_names);
  }
}

/**
 * @brief 析构函数 (Destructor)
 */
template <class ActionT>
BtActionServer<ActionT>::~BtActionServer() {}

/**
 * @brief 配置函数，用于初始化操作 (Configuration function for initialization)
 *
 * @return 成功配置返回 true，否则返回 false (Returns true if configured successfully, otherwise
 * returns false)
 */
template <class ActionT>
bool BtActionServer<ActionT>::on_configure() {
  // 获取节点 (Get the node)
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // 根据 action 名称命名客户端节点 (Name client node after action name)
  std::string client_node_name = action_name_;
  std::replace(client_node_name.begin(), client_node_name.end(), '/', '_');
  // 使用后缀 '_rclcpp_node' 保持参数文件一致性 (Use suffix '_rclcpp_node' to keep parameter file
  // consistency)
  auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args", "-r",
       std::string("__node:=") + std::string(node->get_name()) + "_" + client_node_name +
           "_rclcpp_node",
       "-p",
       "use_sim_time:=" +
           std::string(node->get_parameter("use_sim_time").as_bool() ? "true" : "false"),
       "--"});

  // 支持从 rviz 处理基于主题的目标姿态 (Support for handling the topic-based goal pose from rviz)
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  action_server_ = std::make_shared<ActionServer>(
      node->get_node_base_interface(), node->get_node_clock_interface(),
      node->get_node_logging_interface(), node->get_node_waitables_interface(), action_name_,
      std::bind(&BtActionServer<ActionT>::executeCallback, this));

  // 获取 BT 超时参数 (Get parameters for BT timeouts)
  int timeout;
  node->get_parameter("bt_loop_duration", timeout);
  bt_loop_duration_ = std::chrono::milliseconds(timeout);
  node->get_parameter("default_server_timeout", timeout);
  default_server_timeout_ = std::chrono::milliseconds(timeout);

  // 获取黑板上的错误代码 id 名称 (Get error code id names to grab off of the blackboard)
  error_code_names_ = node->get_parameter("error_code_names").as_string_array();

  // 创建注册自定义节点并执行 BT 的类 (Create the class that registers our custom nodes and executes
  // the BT)
  bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);

  // 创建共享给树中所有节点的黑板 (Create the blackboard that will be shared by all of the nodes in
  // the tree)
  blackboard_ = BT::Blackboard::create();

  // 在黑板上放置项目 (Put items on the blackboard)
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);                         // NOLINT
  blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);      // NOLINT

  return true;
}

/**
 * @brief 激活 BtActionServer 对象，加载行为树并激活动作服务器。
 *        Activate the BtActionServer object, load the behavior tree and activate the action server.
 *
 * @tparam ActionT 动作类型。Action type.
 * @return 是否成功激活。Whether the activation was successful.
 */
template <class ActionT>
bool BtActionServer<ActionT>::on_activate() {
  // 尝试加载默认的行为树 XML 文件。If loading the default behavior tree XML file fails.
  if (!loadBehaviorTree(default_bt_xml_filename_)) {
    // 记录错误信息。Log an error message.
    RCLCPP_ERROR(logger_, "Error loading XML file: %s", default_bt_xml_filename_.c_str());
    return false;
  }
  // 激活动作服务器。Activate the action server.
  action_server_->activate();
  return true;
}

/**
 * @brief 取消激活 BtActionServer 对象。
 *        Deactivate the BtActionServer object.
 *
 * @tparam ActionT 动作类型。Action type.
 * @return 是否成功取消激活。Whether the deactivation was successful.
 */
template <class ActionT>
bool BtActionServer<ActionT>::on_deactivate() {
  // 取消激活动作服务器。Deactivate the action server.
  action_server_->deactivate();
  return true;
}

/**
 * @brief 清理 BtActionServer 对象的资源。
 *        Clean up resources of the BtActionServer object.
 *
 * @tparam ActionT 动作类型。Action type.
 * @return 是否成功清理。Whether the cleanup was successful.
 */
template <class ActionT>
bool BtActionServer<ActionT>::on_cleanup() {
  // 重置客户端节点。Reset the client node.
  client_node_.reset();
  // 重置动作服务器。Reset the action server.
  action_server_.reset();
  // 重置话题记录器。Reset the topic logger.
  topic_logger_.reset();
  // 清除插件库名称。Clear plugin library names.
  plugin_lib_names_.clear();
  // 清除当前行为树 XML 文件名。Clear the current behavior tree XML filename.
  current_bt_xml_filename_.clear();
  // 重置黑板。Reset the blackboard.
  blackboard_.reset();
  // 停止所有行为树上的动作。Halt all actions on the behavior tree.
  bt_->haltAllActions(tree_.rootNode());
  // 重置行为树。Reset the behavior tree.
  bt_.reset();
  return true;
}

/**
 * @brief 加载行为树 (Load the behavior tree)
 *
 * @tparam ActionT 行为类型 (Action type)
 * @param bt_xml_filename 行为树 XML 文件名 (Behavior tree XML filename)
 * @return true 如果加载成功 (If loading is successful)
 * @return false 如果加载失败 (If loading fails)
 */
template <class ActionT>
bool BtActionServer<ActionT>::loadBehaviorTree(const std::string& bt_xml_filename) {
  // 如果文件名为空，使用默认的文件名 (If filename is empty, use the default filename)
  auto filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

  // 如果当前已经是现有的 BT，则使用之前的 BT (If it is the existing BT, use the previous BT)
  if (current_bt_xml_filename_ == filename) {
    RCLCPP_DEBUG(logger_, "BT will not be reloaded as the given xml is already loaded");
    return true;
  }

  // 从指定文件中读取输入 BT XML 到字符串 (Read the input BT XML from the specified file into a
  // string)
  std::ifstream xml_file(filename);

  // 如果文件无法打开，则报错 (If the file cannot be opened, report an error)
  if (!xml_file.good()) {
    RCLCPP_ERROR(logger_, "Couldn't open input XML file: %s", filename.c_str());
    return false;
  }

  // 从 XML 输入创建行为树 (Create the Behavior Tree from the XML input)
  try {
    tree_ = bt_->createTreeFromFile(filename, blackboard_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception when loading BT: %s", e.what());
    return false;
  }

  // 创建 ROS 主题记录器 (Create a ROS topic logger)
  topic_logger_ = std::make_unique<RosTopicLogger>(client_node_, tree_);

  // 更新当前行为树 XML 文件名 (Update the current behavior tree XML filename)
  current_bt_xml_filename_ = filename;
  return true;
}

/**
 * @brief 执行回调函数，处理目标请求，并执行行为树
 * Execute the callback function, handle goal requests, and run the behavior tree
 *
 * @tparam ActionT 行为动作类型
 * Action type
 */
template <class ActionT>
void BtActionServer<ActionT>::executeCallback() {
  // 当收到一个目标请求时，执行 on_goal_received_callback_ 函数
  // 当返回 false 时，终止当前操作并返回
  // When a goal request is received, execute the `on_goal_received_callback_` function
  // Terminate the current operation and return when it returns false
  if (!on_goal_received_callback_(action_server_->get_current_goal())) {
    action_server_->terminate_current();
    return;
  }

  // 定义 is_canceling 函数，用于检查是否需要取消操作
  // Define the `is_canceling` function to check whether the operation needs to be canceled
  auto is_canceling = [&]() {
    // 如果 action_server_ 为空，说明操作服务器不可用，返回 true 取消操作
    // If `action_server_` is null, the action server is not available, return true to cancel the
    // operation
    if (action_server_ == nullptr) {
      RCLCPP_DEBUG(logger_, "Action server unavailable. Canceling.");
      return true;
    }
    // 如果操作服务器处于非活动状态，返回 true 取消操作
    // If the action server is inactive, return true to cancel the operation
    if (!action_server_->is_server_active()) {
      RCLCPP_DEBUG(logger_, "Action server is inactive. Canceling.");
      return true;
    }
    // 返回操作服务器是否收到取消请求
    // Return whether the action server has received a cancel request
    return action_server_->is_cancel_requested();
  };

  // 定义 on_loop 函数，用于处理循环中的事件
  // Define the `on_loop` function to handle events in the loop
  auto on_loop = [&]() {
    // 如果收到抢占请求并且 on_preempt_callback_ 不为空，执行 on_preempt_callback_ 函数
    // If a preempt request is received and `on_preempt_callback_` is not null, execute the
    // `on_preempt_callback_` function
    if (action_server_->is_preempt_requested() && on_preempt_callback_) {
      on_preempt_callback_(action_server_->get_pending_goal());
    }
    topic_logger_->flush();
    on_loop_callback_();
  };

  // 执行在配置步骤中创建的行为树 BT
  // Run the behavior tree (BT) previously created in the configure step
  nav2_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);

  // 确保行为树不处于之前执行的运行状态
  // 注意：如果所有 ControlNodes 都正确实现，这是不需要的。
  // Make sure the behavior tree is not in a running state from a previous execution
  // Note: this is not needed if all ControlNodes are implemented correctly.
  bt_->haltAllActions(tree_.rootNode());

  // 给服务器一个机会填充结果消息或简单地给出操作完成的指示
  // Give the server an opportunity to populate the result message or simply give an indication that
  // the action is complete
  auto result = std::make_shared<typename ActionT::Result>();

  populateErrorCode(result);

  on_completion_callback_(result, rc);

  // 根据行为树的返回状态处理结果
  // Handle the result based on the return status of the behavior tree
  switch (rc) {
    case nav2_behavior_tree::BtStatus::SUCCEEDED:
      RCLCPP_INFO(logger_, "Goal succeeded");
      action_server_->succeeded_current(result);
      break;

    case nav2_behavior_tree::BtStatus::FAILED:
      RCLCPP_ERROR(logger_, "Goal failed");
      action_server_->terminate_current(result);
      break;

    case nav2_behavior_tree::BtStatus::CANCELED:
      RCLCPP_INFO(logger_, "Goal canceled");
      action_server_->terminate_all(result);
      break;
  }
}

/**
 * @brief 为 ActionT 类型的结果填充错误代码（Populate error code for the Result of type ActionT）
 *
 * @tparam ActionT 行动类型（Action Type）
 * @param result 结果指针，用于存储错误代码（Pointer to the result used for storing error code）
 */
template <class ActionT>
void BtActionServer<ActionT>::populateErrorCode(
    typename std::shared_ptr<typename ActionT::Result> result) {
  // 定义最高优先级错误代码变量，并初始化为 int 类型的最大值
  // (Define the highest priority error code variable and initialize it to the maximum value of int)
  int highest_priority_error_code = std::numeric_limits<int>::max();

  // 遍历错误代码名称列表
  // (Iterate through the list of error code names)
  for (const auto& error_code : error_code_names_) {
    try {
      // 从黑板中获取当前错误代码
      // (Get the current error code from the blackboard)
      int current_error_code = blackboard_->get<int>(error_code);

      // 如果当前错误代码不为 0 且小于最高优先级错误代码，则更新最高优先级错误代码
      // (If the current error code is not 0 and is less than the highest priority error code,
      // update the highest priority error code)
      if (current_error_code != 0 && current_error_code < highest_priority_error_code) {
        highest_priority_error_code = current_error_code;
      }
    } catch (...) {
      // 如果无法从黑板获取错误代码，记录错误信息
      // (If unable to get the error code from the blackboard, log the error message)
      RCLCPP_ERROR(logger_, "Failed to get error code: %s from blackboard", error_code.c_str());
    }
  }

  // 如果最高优先级错误代码不等于 int 的最大值，则将其存储到结果中
  // (If the highest priority error code is not equal to the maximum value of int, store it in the
  // result)
  if (highest_priority_error_code != std::numeric_limits<int>::max()) {
    result->error_code = highest_priority_error_code;
  }
}

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_
