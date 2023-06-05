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

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "nav2_behavior_tree/ros_topic_logger.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"

namespace nav2_behavior_tree {
/**
 * @class nav2_behavior_tree::BtActionServer
 * @brief 一个使用行为树执行动作的动作服务器 (An action server that uses behavior tree to execute an
 * action)
 */
template <class ActionT>
class BtActionServer {
public:
  // 使用 nav2_util::SimpleActionServer<ActionT> 作为 ActionServer 的别名
  // (Use nav2_util::SimpleActionServer<ActionT> as an alias for ActionServer)
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  // 当收到目标时的回调函数类型，参数为目标的常量共享指针，返回值为布尔值
  // (Callback type when a goal is received, with parameter as constant shared pointer of the goal,
  // and return value as boolean)
  typedef std::function<bool(typename ActionT::Goal::ConstSharedPtr)> OnGoalReceivedCallback;

  // 在循环中的回调函数类型，无参数
  // (Callback type in the loop, no parameters)
  typedef std::function<void()> OnLoopCallback;

  // 当收到抢占请求时的回调函数类型，参数为目标的常量共享指针
  // (Callback type when a preempt request is received, with parameter as constant shared pointer of
  // the goal)
  typedef std::function<void(typename ActionT::Goal::ConstSharedPtr)> OnPreemptCallback;

  // 当完成时的回调函数类型，参数1为结果的共享指针，参数2为行为树状态 (nav2_behavior_tree::BtStatus)
  // (Callback type on completion, with parameter 1 as shared pointer of the result, and parameter 2
  // as behavior tree status (nav2_behavior_tree::BtStatus))
  typedef std::function<void(typename ActionT::Result::SharedPtr, nav2_behavior_tree::BtStatus)>
      OnCompletionCallback;

  /**
   * @brief 构造函数，用于创建 nav2_behavior_tree::BtActionServer 类的实例 (A constructor for
   * nav2_behavior_tree::BtActionServer class)
   * @param parent 父节点的弱指针 (Weak pointer to the parent LifecycleNode)
   * @param action_name 行为名称 (Name of the action)
   * @param plugin_lib_names 插件库名称列表 (List of plugin library names)
   * @param default_bt_xml_filename 默认行为树 XML 文件名 (Default behavior tree XML filename)
   * @param on_goal_received_callback 当收到目标时的回调函数 (Callback function when a goal is
   * received)
   * @param on_loop_callback 循环时的回调函数 (Callback function during loop)
   * @param on_preempt_callback 当有抢占请求时的回调函数 (Callback function when a preempt request
   * occurs)
   * @param on_completion_callback 当完成时的回调函数 (Callback function when completed)
   */
  explicit BtActionServer(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
      const std::string& action_name,
      const std::vector<std::string>& plugin_lib_names,
      const std::string& default_bt_xml_filename,
      OnGoalReceivedCallback on_goal_received_callback,
      OnLoopCallback on_loop_callback,
      OnPreemptCallback on_preempt_callback,
      OnCompletionCallback on_completion_callback);

  /**
   * @brief 析构函数，用于销毁 nav2_behavior_tree::BtActionServer 类的实例 (A destructor for
   * nav2_behavior_tree::BtActionServer class)
   */
  ~BtActionServer();

  /**
   * @brief 配置成员变量 (Configures member variables)
   * 初始化行为服务器，从 XML 文件构建行为树，并调用用户定义的 onConfigure 函数 (Initializes action
   * server, builds behavior tree from xml file, and calls user-defined onConfigure)
   * @return bool 如果成功返回 true，失败返回 false (true on SUCCESS and false on FAILURE)
   */
  bool on_configure();

  /**
   * @brief 激活行为服务器 (Activates the action server)
   * @return bool 如果成功返回 true，失败返回 false (true on SUCCESS and false on FAILURE)
   */
  bool on_activate();

  /**
   * @brief 停用行为服务器 (Deactivates the action server)
   * @return bool 如果成功返回 true，失败返回 false (true on SUCCESS and false on FAILURE)
   */
  bool on_deactivate();

  /**
   * @brief 重置成员变量 (Resets member variables)
   * @return bool 如果成功返回 true，失败返回 false (true on SUCCESS and false on FAILURE)
   */
  bool on_cleanup();

  /**
   * @brief 替换当前的行为树 (BT) 为另一个
   * @brief Replace current BT with another one
   * @param bt_xml_filename 包含新 BT 的文件，如果为空则使用默认文件名
   * @param bt_xml_filename The file containing the new BT, uses default filename if empty
   * @return bool 如果生成的 BT 对应于 bt_xml_filename 中的 BT，则返回 true。如果出错，则返回
   * false，并保留先前的 BT
   * @return bool true if the resulting BT correspond to the one in bt_xml_filename. false
   * if something went wrong, and previous BT is maintained
   */
  bool loadBehaviorTree(const std::string& bt_xml_filename = "");

  /**
   * @brief 获取 BT 黑板的 Getter 函数
   * @brief Getter function for BT Blackboard
   * @return BT::Blackboard::Ptr 指向当前 BT 黑板的共享指针
   * @return BT::Blackboard::Ptr Shared pointer to current BT blackboard
   */
  BT::Blackboard::Ptr getBlackboard() const { return blackboard_; }

  /**
   * @brief 获取当前 BT XML 文件名的 Getter 函数
   * @brief Getter function for current BT XML filename
   * @return string 包含当前 BT XML 文件名的字符串
   * @return string Containing current BT XML filename
   */
  std::string getCurrentBTFilename() const { return current_bt_xml_filename_; }

  /**
   * @brief 获取默认 BT XML 文件名的 Getter 函数
   * @brief Getter function for default BT XML filename
   * @return string 包含默认 BT XML 文件名的字符串
   * @return string Containing default BT XML filename
   */
  std::string getDefaultBTFilename() const { return default_bt_xml_filename_; }

  /**
   * @brief 如果请求抢占，则接受待处理目标的包装函数
   * @brief Wrapper function to accept pending goal if a preempt has been requested
   * @return 指向待处理动作目标的共享指针
   * @return Shared pointer to pending action goal
   */
  const std::shared_ptr<const typename ActionT::Goal> acceptPendingGoal() {
    return action_server_->accept_pending_goal();
  }

  /**
   * @brief 如果请求抢占，则终止待处理目标的包装函数
   * @brief Wrapper function to terminate pending goal if a preempt has been requested
   */
  void terminatePendingGoal() { action_server_->terminate_pending_goal(); }

  /**
   * @brief 获取当前目标的包装函数
   * @brief Wrapper function to get current goal
   * @return 指向当前动作目标的共享指针
   * @return Shared pointer to current action goal
   */
  const std::shared_ptr<const typename ActionT::Goal> getCurrentGoal() const {
    return action_server_->get_current_goal();
  }

  /**
   * @brief 获取待处理目标的包装函数 (Wrapper function to get pending goal)
   * @return 返回指向待处理动作目标的共享指针 (Shared pointer to pending action goal)
   */
  const std::shared_ptr<const typename ActionT::Goal> getPendingGoal() const {
    // 调用 action_server_ 的 get_pending_goal() 方法获取待处理目标 (Call the get_pending_goal()
    // method of action_server_ to get the pending goal)
    return action_server_->get_pending_goal();
  }

  /**
   * @brief 发布动作反馈的包装函数 (Wrapper function to publish action feedback)
   * @param feedback 指向动作反馈的共享指针 (Shared pointer to action feedback)
   */
  void publishFeedback(typename std::shared_ptr<typename ActionT::Feedback> feedback) {
    // 调用 action_server_ 的 publish_feedback() 方法发布反馈 (Call the publish_feedback() method of
    // action_server_ to publish the feedback)
    action_server_->publish_feedback(feedback);
  }

  /**
   * @brief 获取当前 BT 树的 getter 函数 (Getter function for the current BT tree)
   * @return 返回当前行为树 (BT::Tree Current behavior tree)
   */
  const BT::Tree& getTree() const { return tree_; }

  /**
   * @brief 停止当前树的函数。它将通过调用其 halt() 实现中断正在运行节点的执行（仅适用于可能返回
   * RUNNING 的 Async 节点）(Function to halt the current tree. It will interrupt the execution of
   * RUNNING nodes by calling their halt() implementation (only for Async nodes that may return
   * RUNNING))
   */
  void haltTree() { tree_.rootNode()->halt(); }

protected:
  /**
   * @brief Action server callback（动作服务器回调）
   */
  void executeCallback();

  /**
   * @brief updates the action server result to the highest priority error code posted on the
   * blackboard（将动作服务器结果更新为黑板上发布的最高优先级错误代码）
   * @param result the action server result to be updated（要更新的动作服务器结果）
   */
  void populateErrorCode(typename std::shared_ptr<typename ActionT::Result> result);

  // Action name（动作名称）
  std::string action_name_;

  // Our action server implements the template action（我们的动作服务器实现了模板动作）
  std::shared_ptr<ActionServer> action_server_;

  // Behavior Tree to be executed when goal is received（收到目标时要执行的行为树）
  BT::Tree tree_;

  // The blackboard shared by all of the nodes in the tree（树中所有节点共享的黑板）
  BT::Blackboard::Ptr blackboard_;

  // The XML file that contains the Behavior Tree to create（包含要创建的行为树的XML文件）
  std::string current_bt_xml_filename_;
  std::string default_bt_xml_filename_;

  // The wrapper class for the BT functionality（BT功能的包装类）
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;

  // Libraries to pull plugins (BT Nodes) from（从中提取插件（BT节点）的库）
  std::vector<std::string> plugin_lib_names_;

  // Error code id names（错误代码ID名称）
  std::vector<std::string> error_code_names_;

  // A regular, non-spinning ROS node that we can use for calls to the action
  // client（可用于调用动作客户端的常规非旋转ROS节点）
  rclcpp::Node::SharedPtr client_node_;

  // Parent node（父节点）
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // Clock（时钟）
  rclcpp::Clock::SharedPtr clock_;

  // Logger（记录器）
  rclcpp::Logger logger_{rclcpp::get_logger("BtActionServer")};

  // To publish BT logs（发布BT日志）
  std::unique_ptr<RosTopicLogger> topic_logger_;

  // Duration for each iteration of BT execution（每次BT执行的持续时间）
  std::chrono::milliseconds bt_loop_duration_;

  // Default timeout value while waiting for response from a server（等待服务器响应时的默认超时值）
  std::chrono::milliseconds default_server_timeout_;

  // User-provided callbacks（用户提供的回调）
  OnGoalReceivedCallback on_goal_received_callback_;
  OnLoopCallback on_loop_callback_;
  OnPreemptCallback on_preempt_callback_;
  OnCompletionCallback on_completion_callback_;
};

}  // namespace nav2_behavior_tree

#include <nav2_behavior_tree/bt_action_server_impl.hpp>  // NOLINT(build/include_order)
#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_HPP_
