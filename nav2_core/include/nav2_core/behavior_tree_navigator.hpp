// Copyright (c) 2021-2023 Samsung Research America
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

#ifndef NAV2_CORE__BEHAVIOR_TREE_NAVIGATOR_HPP_
#define NAV2_CORE__BEHAVIOR_TREE_NAVIGATOR_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_behavior_tree/bt_action_server.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_core {

/**
 * @struct FeedbackUtils
 * @brief 导航器反馈实用程序，用于获取变换和参考帧。
 * @brief Navigator feedback utilities required to get transforms and reference frames.
 */
struct FeedbackUtils {
  // 机器人的参考帧名称
  // The robot frame name
  std::string robot_frame;

  // 全局参考帧名称
  // The global frame name
  std::string global_frame;

  // 变换公差
  // The transform tolerance
  double transform_tolerance;

  // tf2_ros::Buffer 的共享指针，用于处理坐标变换
  // A shared pointer to tf2_ros::Buffer for handling coordinate transforms
  std::shared_ptr<tf2_ros::Buffer> tf;
};

/**
 * @class NavigatorMuxer
 * @brief 通过一次只处理一个插件来控制 BT 导航器状态的类。
 * @brief A class to control the state of the BT navigator by allowing only a single
 * plugin to be processed at a time.
 */
class NavigatorMuxer {
public:
  /**
   * @brief 导航器多路复用器构造函数。
   * @brief A Navigator Muxer constructor.
   */
  NavigatorMuxer() : current_navigator_(std::string("")) {}

  /**
   * @brief 获取导航器复用器状态 (Get the navigator muxer state)
   * @return bool 如果正在进行导航，则返回true (If a navigator is in progress, return true)
   */
  bool isNavigating() {
    std::scoped_lock l(mutex_);  // 使用scoped_lock锁定互斥体 (Lock the mutex with scoped_lock)
    return !current_navigator_.empty();  // 如果当前导航器不为空，则返回true (Return true if the
                                         // current navigator is not empty)
  }

  /**
   * @brief 使用给定的导航器开始导航 (Start navigating with a given navigator)
   * @param string 要启动的导航器的名称 (Name of the navigator to start)
   */
  void startNavigating(const std::string& navigator_name) {
    std::scoped_lock l(mutex_);  // 使用scoped_lock锁定互斥体 (Lock the mutex with scoped_lock)
    if (!current_navigator_
             .empty()) {  // 如果当前导航器不为空 (If the current navigator is not empty)
      RCLCPP_ERROR(
          rclcpp::get_logger("NavigatorMutex"),
          "Major error! Navigation requested while another navigation"
          " task is in progress! This likely occurred from an incorrect"
          "implementation of a navigator plugin.");
    }
    current_navigator_ = navigator_name;  // 将当前导航器设置为给定的导航器名称 (Set the current
                                          // navigator to the given navigator name)
  }

  /**
   * @brief 使用给定的导航器停止导航 (Stop navigating with a given navigator)
   * @param string 结束任务的导航器名称
   */
  void stopNavigating(const std::string& navigator_name) {
    std::scoped_lock l(mutex_);
    if (current_navigator_ != navigator_name) {
      RCLCPP_ERROR(
          rclcpp::get_logger("NavigatorMutex"),
          "Major error! Navigation stopped while another navigation"
          " task is in progress! This likely occurred from an incorrect"
          "implementation of a navigator plugin.");
    } else {
      current_navigator_ = std::string("");
    }
  }

protected:
  std::string current_navigator_;
  std::mutex mutex_;
};

/**
 * @class NavigatorBase
 * @brief 导航器接口，允许将导航器存储在向量中并通过 pluginlib
 * 进行访问，因为它们是模板。这些功能将由 BehaviorTreeNavigator 实现，而不是用户。用户应该实现来自
 * BehaviorTreeNavigator 的虚拟方法以实现他们的导航器动作。
 * @brief Navigator interface to allow navigators to be stored in a vector and accessed via
 * pluginlib due to templates. These functions will be implemented by BehaviorTreeNavigator, not the
 * user. The user should implement the virtual methods from BehaviorTreeNavigator to implement their
 * navigator action.
 */
class NavigatorBase {
public:
  // 默认构造函数
  // Default constructor
  NavigatorBase() = default;

  // 默认析构函数
  // Default destructor
  ~NavigatorBase() = default;

  /**
   * @brief 配置导航器的后端 BT 和操作 (Configuration of the navigator's backend BT and actions)
   * @param parent_node 父节点的弱指针 (Weak pointer to the parent node)
   * @param plugin_lib_names 插件库名字列表 (List of plugin library names)
   * @param feedback_utils 反馈工具类实例 (Instance of the FeedbackUtils class)
   * @param plugin_muxer 导航器插件复用器指针 (Pointer to the NavigatorMuxer for plugins)
   * @param odom_smoother 里程计平滑器的共享指针 (Shared pointer to the OdomSmoother)
   * @return bool 如果成功返回 true (If successful, returns true)
   */
  virtual bool on_configure(
      rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
      const std::vector<std::string>& plugin_lib_names,
      const FeedbackUtils& feedback_utils,
      nav2_core::NavigatorMuxer* plugin_muxer,
      std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) = 0;

  /**
   * @brief 激活导航器的后端 BT 和操作 (Activation of the navigator's backend BT and actions)
   * @return bool 如果成功返回 true (If successful, returns true)
   */
  virtual bool on_activate() = 0;

  /**
   * @brief 停用导航器的后端 BT 和操作 (Deactivation of the navigator's backend BT and actions)
   * @return bool 如果成功返回 true (If successful, returns true)
   */
  virtual bool on_deactivate() = 0;

  /**
   * @brief 清理导航器 (Cleanup a navigator)
   * @return bool 如果成功返回 true (If successful, returns true)
   */
  virtual bool on_cleanup() = 0;
};

/**
 * @class BehaviorTreeNavigator
 * @brief 导航器接口，作为所有基于 BT 的导航器动作插件的基类（Navigator interface that acts as a
 * base class for all BT-based Navigator action's plugins） 所有来自 NavigatorBase 的方法都被标记为
 * final，因此它们不能被派生方法覆盖（instead, users should use the appropriate APIs provided after
 * BT Action handling）。
 */
template <class ActionT>
class BehaviorTreeNavigator : public NavigatorBase {
public:
  // 使用 Ptr 类型定义一个指向 nav2_core::BehaviorTreeNavigator<ActionT> 的共享指针类型（Using Ptr
  // type to define a shared pointer type pointing to nav2_core::BehaviorTreeNavigator<ActionT>）
  using Ptr = std::shared_ptr<nav2_core::BehaviorTreeNavigator<ActionT>>;

  /**
   * @brief 导航器构造函数（A Navigator constructor）
   */
  BehaviorTreeNavigator() : NavigatorBase() {
    // 初始化 plugin_muxer_ 为空指针（Initialize plugin_muxer_ as a nullptr）
    plugin_muxer_ = nullptr;
  }

  /**
   * @brief 虚拟析构函数
   * @brief Virtual destructor
   */
  virtual ~BehaviorTreeNavigator() = default;

  /**
   * @brief 用于设置导航器后端 BT 和操作的配置
   * @brief Configuration to setup the navigator's backend BT and actions
   * @param parent_node 要使用的 ROS 父节点
   * @param plugin_lib_names 要加载的插件共享库的向量
   * @param feedback_utils 对导航器有用的一些实用程序
   * @param plugin_muxer 用于确保一次只能激活一个导航器的复用对象
   * @param odom_smoother 获取当前平滑机器人速度的对象
   * @return bool 如果成功
   *
   * @param parent_node The ROS parent node to utilize
   * @param plugin_lib_names a vector of plugin shared libraries to load
   * @param feedback_utils Some utilities useful for navigators to have
   * @param plugin_muxer The muxing object to ensure only one navigator
   * can be active at a time
   * @param odom_smoother Object to get current smoothed robot's speed
   * @return bool If successful
   */
  bool on_configure(
      rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
      const std::vector<std::string>& plugin_lib_names,
      const FeedbackUtils& feedback_utils,
      nav2_core::NavigatorMuxer* plugin_muxer,
      std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) final {
    // 锁定父节点并获取日志记录器和时钟对象
    // Lock the parent node and get the logger and clock objects
    auto node = parent_node.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // 设置反馈实用程序和插件复用器
    // Set the feedback utilities and plugin muxer
    feedback_utils_ = feedback_utils;
    plugin_muxer_ = plugin_muxer;

    // 获取此导航器的默认行为树
    // Get the default behavior tree for this navigator
    std::string default_bt_xml_filename = getDefaultBTFilepath(parent_node);

    // 为此导航器创建行为树操作服务器
    // Create the Behavior Tree Action Server for this navigator
    bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ActionT>>(
        node, getName(), plugin_lib_names, default_bt_xml_filename,
        std::bind(&BehaviorTreeNavigator::onGoalReceived, this, std::placeholders::_1),
        std::bind(&BehaviorTreeNavigator::onLoop, this),
        std::bind(&BehaviorTreeNavigator::onPreempt, this, std::placeholders::_1),
        std::bind(
            &BehaviorTreeNavigator::onCompletion, this, std::placeholders::_1,
            std::placeholders::_2));

    // 检查配置是否成功
    // Check if configuration is successful
    bool ok = true;
    if (!bt_action_server_->on_configure()) {
      ok = false;
    }

    // 获取行为树操作服务器的黑板并设置相关参数
    // Get the blackboard of the behavior tree action server and set related parameters
    BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", feedback_utils.tf);  // NOLINT
    blackboard->set<bool>("initial_pose_received", false);                              // NOLINT
    blackboard->set<int>("number_recoveries", 0);                                       // NOLINT
    blackboard->set<std::shared_ptr<nav2_util::OdomSmoother>>(
        "odom_smoother", odom_smoother);                                                // NOLINT

    // 返回配置结果
    // Return the configuration result
    return configure(parent_node, odom_smoother) && ok;
  }

  /**
   * @brief 激活导航器后端 BT 和操作 (Activation of the navigator's backend BT and actions)
   * @return bool 如果成功返回 true (If successful, returns true)
   */
  bool on_activate() final {
    bool ok = true;

    // 尝试激活 bt_action_server_，如果失败则将 ok 设置为 false
    // (Attempt to activate bt_action_server_, if it fails, set ok to false)
    if (!bt_action_server_->on_activate()) {
      ok = false;
    }

    // 返回激活状态和 ok 的逻辑与结果
    // (Return the logical AND result of activation status and ok)
    return activate() && ok;
  }

  /**
   * @brief 取消激活导航器后端 BT 和操作 (Deactivation of the navigator's backend BT and actions)
   * @return bool 如果成功返回 true (If successful, returns true)
   */
  bool on_deactivate() final {
    bool ok = true;

    // 尝试取消激活 bt_action_server_，如果失败则将 ok 设置为 false
    // (Attempt to deactivate bt_action_server_, if it fails, set ok to false)
    if (!bt_action_server_->on_deactivate()) {
      ok = false;
    }

    // 返回取消激活状态和 ok 的逻辑与结果
    // (Return the logical AND result of deactivation status and ok)
    return deactivate() && ok;
  }

  /**
   * @brief 清理导航器 (Cleanup a navigator)
   * @return bool 如果成功返回 true (If successful, returns true)
   */
  bool on_cleanup() final {
    bool ok = true;

    // 尝试清理 bt_action_server_，如果失败则将 ok 设置为 false
    // (Attempt to cleanup bt_action_server_, if it fails, set ok to false)
    if (!bt_action_server_->on_cleanup()) {
      ok = false;
    }

    // 重置 bt_action_server_ 对象
    // (Reset the bt_action_server_ object)
    bt_action_server_.reset();

    // 返回清理状态和 ok 的逻辑与结果
    // (Return the logical AND result of cleanup status and ok)
    return cleanup() && ok;
  }

  // 获取默认 BT 文件路径的纯虚函数，需要在派生类中实现
  // (Pure virtual function for getting the default BT filepath, needs to be implemented in derived
  // classes)
  virtual std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) = 0;

  /**
   * @brief 获取此导航器要公开的动作名称 (Get the action name of this navigator to expose)
   * @return string 要公开的动作名称 (Name of action to expose)
   */
  virtual std::string getName() = 0;

protected:
  /**
   * @brief 用于复用导航器的中间目标接收功能 (An intermediate goal reception function to mux
   * navigators)
   */
  bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal) {
    // 如果插件复用器正在导航，则返回错误日志 (If the plugin muxer is navigating, return an error
    // log)
    if (plugin_muxer_->isNavigating()) {
      RCLCPP_ERROR(
          logger_,
          "从 %s 请求导航，而另一个导航器正在处理，拒绝请求。"
          "Requested navigation from %s while another navigator is processing,"
          " rejecting request.",
          getName().c_str());
      return false;
    }

    // 接收目标并检查是否被接受 (Receive the goal and check if it's accepted)
    bool goal_accepted = goalReceived(goal);

    // 如果目标被接受，启动导航 (If the goal is accepted, start navigating)
    if (goal_accepted) {
      plugin_muxer_->startNavigating(getName());
    }

    return goal_accepted;
  }

  /**
   * @brief 用于复用导航器的中间完成功能 (An intermediate completion function to mux navigators)
   */
  void onCompletion(
      typename ActionT::Result::SharedPtr result,
      const nav2_behavior_tree::BtStatus final_bt_status) {
    // 停止导航 (Stop navigating)
    plugin_muxer_->stopNavigating(getName());
    // 完成目标 (Complete the goal)
    goalCompleted(result, final_bt_status);
  }

  /**
   * @brief 当 BT 动作服务器接收到新目标时调用的回调 (A callback to be called when a new goal is
   * received by the BT action server) 可用于检查目标是否有效并根据接收到的目标放置依赖项 (Can be
   * used to check if goal is valid and put values on the blackboard which depend on the received
   * goal)
   */
  virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief 定义在 BT 中进行一次迭代时发生的执行的回调 (A callback that defines execution that
   * happens on one iteration through the BT) 可用于发布动作反馈 (Can be used to publish action
   * feedback)
   */
  virtual void onLoop() = 0;

  /**
   * @brief 当请求抢占时调用的回调 (A callback that is called when a preempt is requested)
   */
  virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief 当动作完成时调用的回调；可以填充动作结果消息或指示此动作已完成 (A callback that is
   * called when the action is completed; Can fill in action result message or indicate that this
   * action is done)
   */
  virtual void goalCompleted(
      typename ActionT::Result::SharedPtr result,
      const nav2_behavior_tree::BtStatus final_bt_status) = 0;

  /**
   * @param 配置资源的方法 (Method to configure resources)
   */
  virtual bool configure(
      rclcpp_lifecycle::LifecycleNode::WeakPtr /*node*/,
      std::shared_ptr<nav2_util::OdomSmoother> /*odom_smoother*/) {
    return true;
  }

  /**
   * @brief 清理资源的方法 (Method to cleanup resources)
   */
  virtual bool cleanup() { return true; }

  /**
   * @brief 激活任何涉及执行的线程的方法 (Method to activate any threads involved in execution)
   */
  virtual bool activate() { return true; }

  /**
   * @brief 停用任何涉及执行的线程的方法 (Method to deactivate and any threads involved in
   * execution)
   */
  virtual bool deactivate() { return true; }

  std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
  rclcpp::Logger logger_{rclcpp::get_logger("Navigator")};
  rclcpp::Clock::SharedPtr clock_;
  FeedbackUtils feedback_utils_;
  NavigatorMuxer* plugin_muxer_;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__BEHAVIOR_TREE_NAVIGATOR_HPP_
