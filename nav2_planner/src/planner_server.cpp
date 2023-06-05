// Copyright (c) 2018 Intel Corporation
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

#include "nav2_planner/planner_server.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_planner {

/**
 * @brief PlannerServer的构造函数，继承自nav2_util::LifecycleNode
 * @param options 节点选项
 * @details
 * 初始化全局规划器加载器gp_loader_、默认规划器名称default_ids_和默认规划器类型default_types_，
 *          并声明节点参数planner_plugins和expected_planner_frequency，
 *          如果planner_plugins与default_ids_相等，则为每个默认规划器名称声明一个对应的规划器类型参数。
 *          初始化全局代价地图costmap_ros_，并启动一个线程运行代价地图节点。
 */
PlannerServer::PlannerServer(const rclcpp::NodeOptions &options)
    : nav2_util::LifecycleNode(
          "planner_server", "", options),  // 调用基类构造函数，设置节点名称为"planner_server"
      gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),  // 初始化全局规划器加载器gp_loader_
      default_ids_{"GridBased"},               // 设置默认规划器名称为"GridBased"
      default_types_{
          "nav2_navfn_planner/NavfnPlanner"},  // 设置默认规划器类型为"nav2_navfn_planner/NavfnPlanner"
      costmap_(nullptr) {                      // 初始化成员变量costmap_

  RCLCPP_INFO(get_logger(), "Creating");  // 输出日志信息

  // Declare this node's parameters
  declare_parameter(
      "planner_plugins", default_ids_);  // 声明节点参数"planner_plugins"，默认值为"default_ids_"
  declare_parameter(
      "expected_planner_frequency", 1.0);  // 声明节点参数"expected_planner_frequency"，默认值为1.0

  get_parameter("planner_plugins", planner_ids_);  // 获取节点参数"planner_plugins"的值
  if (planner_ids_ == default_ids_) {  // 如果"planner_plugins"的值与"default_ids_"相等
    for (size_t i = 0; i < default_ids_.size(); ++i) {  // 遍历默认规划器名称
      declare_parameter(
          default_ids_[i] + ".plugin",
          default_types_[i]);  // 为每个默认规划器名称声明一个对应的规划器类型参数
    }
  }

  // Setup the global costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(  // 创建代价地图ROS节点
      "global_costmap", std::string{get_namespace()},
      "global_costmap",  // 设置节点名称、命名空间和代价地图名称
      get_parameter("use_sim_time")
          .as_bool());  // 获取节点参数"use_sim_time"的值，并传递给代价地图ROS节点
  // Launch a thread to run the costmap node
  costmap_thread_ =
      std::make_unique<nav2_util::NodeThread>(costmap_ros_);  // 启动一个线程运行代价地图ROS节点
}

/**
 * @brief PlannerServer的析构函数
 * @details 清空规划器planners_，并重置costmap_thread_
 */
PlannerServer::~PlannerServer() {
  /*
   * Backstop ensuring this state is destroyed, even if deactivate/cleanup are
   * never called.
   */
  planners_.clear();        // 清空规划器planners_
  costmap_thread_.reset();  // 重置costmap_thread_
}

/**
 * @brief 当组件被配置时调用的函数，主要作用是初始化全局规划器。
 *
 * @param state 生命周期状态
 * @return nav2_util::CallbackReturn 返回成功或失败
 */
nav2_util::CallbackReturn PlannerServer::on_configure(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Configuring");

  // 配置 costmap_ros_ 对象
  costmap_ros_->configure();

  // 获取 costmap 对象
  costmap_ = costmap_ros_->getCostmap();

  // 打印 costmap 的大小
  RCLCPP_DEBUG(
      get_logger(), "Costmap size: %d,%d", costmap_->getSizeInCellsX(),
      costmap_->getSizeInCellsY());

  // 获取 tf_buffer 对象
  tf_ = costmap_ros_->getTfBuffer();

  // 初始化 planner_types_ 数组
  planner_types_.resize(planner_ids_.size());

  // 循环遍历所有的 planner_ids_
  auto node = shared_from_this();
  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      // 获取 planner_ids_ 对应的插件类型
      planner_types_[i] = nav2_util::get_plugin_type_param(node, planner_ids_[i]);

      // 创建 GlobalPlanner 对象
      nav2_core::GlobalPlanner::Ptr planner = gp_loader_.createUniqueInstance(planner_types_[i]);

      // 打印创建的插件信息
      RCLCPP_INFO(
          get_logger(), "Created global planner plugin %s of type %s", planner_ids_[i].c_str(),
          planner_types_[i].c_str());

      // 配置 GlobalPlanner 对象
      planner->configure(node, planner_ids_[i], tf_, costmap_ros_);

      // 将 GlobalPlanner 对象插入到 planners_ 中
      planners_.insert({planner_ids_[i], planner});
    } catch (const pluginlib::PluginlibException &ex) {
      // 如果创建 GlobalPlanner 失败，打印错误信息并返回失败
      RCLCPP_FATAL(get_logger(), "Failed to create global planner. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  // 拼接所有的 planner_ids_
  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  // 打印可用的 planner_ids_
  RCLCPP_INFO(
      get_logger(), "Planner Server has %s planners available.", planner_ids_concat_.c_str());

  // 获取 expected_planner_frequency 参数，并计算 max_planner_duration_
  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    // 如果 expected_planner_frequency 参数小于等于 0，则打印警告信息
    RCLCPP_WARN(
        get_logger(),
        "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
        " than 0.0 to turn on duration overrrun warning messages",
        expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  // 初始化发布器
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  // 创建 action servers
  action_server_pose_ = std::make_unique<ActionServerToPose>(
      shared_from_this(), "compute_path_to_pose", std::bind(&PlannerServer::computePlan, this),
      nullptr, std::chrono::milliseconds(500), true);

  action_server_poses_ = std::make_unique<ActionServerThroughPoses>(
      shared_from_this(), "compute_path_through_poses",
      std::bind(&PlannerServer::computePlanThroughPoses, this), nullptr,
      std::chrono::milliseconds(500), true);

  return nav2_util::CallbackReturn::SUCCESS;
}

/**
 * @brief 激活回调函数，用于在组件激活时执行一些初始化操作
 * @param state 组件状态，类型为 rclcpp_lifecycle::State，未使用
 * @return nav2_util::CallbackReturn::SUCCESS 表示激活成功
 *
 * 当 PlannerServer 组件被激活时，会调用该回调函数，进行一些必要的初始化操作。
 * 具体包括：
 * 1. 激活 plan_publisher_ 发布者；
 * 2. 激活 action_server_pose_ 行为服务器；
 * 3. 激活 action_server_poses_ 行为服务器；
 * 4. 激活 costmap_ros_ 实例；
 * 5. 遍历所有的规划器实例，并逐个激活；
 * 6. 创建 is_path_valid 服务，用于检查路径是否有效；
 * 7. 添加动态参数回调函数；
 * 8. 创建 bond 连接。
 *
 * 返回 CallbackReturn::SUCCESS 表示激活成功。
 */
nav2_util::CallbackReturn PlannerServer::on_activate(const rclcpp_lifecycle::State & /*state*/) {
  // 输出日志信息，表示正在激活 PlannerServer 组件
  RCLCPP_INFO(get_logger(), "Activating");

  // 激活 plan_publisher_ 发布者
  plan_publisher_->on_activate();
  // 激活 action_server_pose_ 行为服务器
  action_server_pose_->activate();
  // 激活 action_server_poses_ 行为服务器
  action_server_poses_->activate();
  // 激活 costmap_ros_ 实例
  costmap_ros_->activate();

  // 遍历所有的规划器实例，并逐个激活
  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->activate();
  }

  // 获取当前节点的 shared_ptr 对象
  auto node = shared_from_this();

  // 创建 is_path_valid 服务，用于检查路径是否有效
  is_path_valid_service_ = node->create_service<nav2_msgs::srv::IsPathValid>(
      "is_path_valid",
      std::bind(&PlannerServer::isPathValid, this, std::placeholders::_1, std::placeholders::_2));

  // 添加动态参数回调函数
  dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(&PlannerServer::dynamicParametersCallback, this, _1));

  // 创建 bond 连接
  createBond();

  // 返回 CallbackReturn::SUCCESS 表示激活成功
  return nav2_util::CallbackReturn::SUCCESS;
}

/**
 * @brief 当 PlannerServer 被 deactive 时的回调函数
 * @param state 生命周期状态
 * @details 停用 action server 和 plan publisher，同时停用 costmap 和所有的 planner。
 *          销毁 bond 连接，并返回成功的回调结果。
 */
nav2_util::CallbackReturn PlannerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  // 停用 action server
  action_server_pose_->deactivate();
  action_server_poses_->deactivate();

  // 停用 plan publisher
  plan_publisher_->on_deactivate();

  /*
   * The costmap is also a lifecycle node, so it may have already fired on_deactivate
   * via rcl preshutdown cb. Despite the rclcpp docs saying on_shutdown callbacks fire
   * in the order added, the preshutdown callbacks clearly don't per se, due to using an
   * unordered_set iteration. Once this issue is resolved, we can maybe make a stronger
   * ordering assumption: https://github.com/ros2/rclcpp/issues/2096
   */

  // 如果 costmap 还处于激活状态，则停用它
  if (costmap_ros_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    costmap_ros_->deactivate();
  }

  // 停用所有的 planner
  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->deactivate();
  }

  // 重置动态参数处理器
  dyn_params_handler_.reset();

  // 销毁 bond 连接
  destroyBond();

  // 返回成功的回调结果
  return nav2_util::CallbackReturn::SUCCESS;
}

/**
 * @brief 清理函数回调，用于清理 PlannerServer 中的各个组件
 * @param state 生命周期状态
 * @details 该函数用于清理 PlannerServer 中的各个组件，包括 action_server_pose_,
 * action_server_poses_, plan_publisher_, tf_ 等。 同时检查 costmap_ros_ 的状态是否为
 * PRIMARY_STATE_INACTIVE，如果是则调用 costmap_ros_->cleanup() 函数进行清理。 最后清理 planners_
 * 中的所有规划器，并将其清空。返回 nav2_util::CallbackReturn::SUCCESS 表示成功。
 */

nav2_util::CallbackReturn PlannerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // 重置 action_server_pose_, action_server_poses_, plan_publisher_, tf_
  action_server_pose_.reset();
  action_server_poses_.reset();
  plan_publisher_.reset();
  tf_.reset();

  /*
   * Double check whether something else transitioned it to INACTIVE
   * already, e.g. the rcl preshutdown callback.
   */

  // 检查 costmap_ros_ 的状态是否为 PRIMARY_STATE_INACTIVE，如果是则调用 costmap_ros_->cleanup()
  // 进行清理
  if (costmap_ros_->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    costmap_ros_->cleanup();
  }

  // 清理 planners_ 中的所有规划器，并将其清空
  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->cleanup();
  }

  planners_.clear();
  costmap_thread_.reset();
  costmap_ = nullptr;
  return nav2_util::CallbackReturn::SUCCESS;
}

/**
 * @brief 关闭函数回调，用于关闭 PlannerServer
 * @param state 生命周期状态
 * @details 该函数用于关闭 PlannerServer，并返回 nav2_util::CallbackReturn::SUCCESS 表示成功。
 */
nav2_util::CallbackReturn PlannerServer::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

template <typename T>
bool PlannerServer::isServerInactive(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server) {
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}

void PlannerServer::waitForCostmap() {
  // Don't compute a plan until costmap is valid (after clear costmap)
  rclcpp::Rate r(100);
  while (!costmap_ros_->isCurrent()) {
    r.sleep();
  }
}

template <typename T>
bool PlannerServer::isCancelRequested(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server) {
  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }

  return false;
}

template <typename T>
void PlannerServer::getPreemptedGoalIfRequested(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server,
    typename std::shared_ptr<const typename T::Goal> goal) {
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template <typename T>
bool PlannerServer::getStartPose(
    typename std::shared_ptr<const typename T::Goal> goal, geometry_msgs::msg::PoseStamped &start) {
  if (goal->use_start) {
    start = goal->start;
  } else if (!costmap_ros_->getRobotPose(start)) {
    return false;
  }

  return true;
}

bool PlannerServer::transformPosesToGlobalFrame(
    geometry_msgs::msg::PoseStamped &curr_start, geometry_msgs::msg::PoseStamped &curr_goal) {
  if (!costmap_ros_->transformPoseToGlobalFrame(curr_start, curr_start) ||
      !costmap_ros_->transformPoseToGlobalFrame(curr_goal, curr_goal)) {
    return false;
  }

  return true;
}

template <typename T>
bool PlannerServer::validatePath(
    const geometry_msgs::msg::PoseStamped &goal,
    const nav_msgs::msg::Path &path,
    const std::string &planner_id) {
  if (path.poses.empty()) {
    RCLCPP_WARN(
        get_logger(),
        "Planning algorithm %s failed to generate a valid"
        " path to (%.2f, %.2f)",
        planner_id.c_str(), goal.pose.position.x, goal.pose.position.y);
    return false;
  }

  RCLCPP_DEBUG(
      get_logger(), "Found valid path of size %zu to (%.2f, %.2f)", path.poses.size(),
      goal.pose.position.x, goal.pose.position.y);

  return true;
}

void PlannerServer::computePlanThroughPoses() {
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = steady_clock_.now();

  // Initialize the ComputePathThroughPoses goal and result
  auto goal = action_server_poses_->get_current_goal();
  auto result = std::make_shared<ActionThroughPoses::Result>();
  nav_msgs::msg::Path concat_path;

  geometry_msgs::msg::PoseStamped curr_start, curr_goal;

  try {
    if (isServerInactive(action_server_poses_) || isCancelRequested(action_server_poses_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_poses_, goal);

    if (goal->goals.empty()) {
      throw nav2_core::NoViapointsGiven("No viapoints given");
    }

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose<ActionThroughPoses>(goal, start)) {
      throw nav2_core::PlannerTFError("Unable to get start pose");
    }

    // Get consecutive paths through these points
    for (unsigned int i = 0; i != goal->goals.size(); i++) {
      // Get starting point
      if (i == 0) {
        curr_start = start;
      } else {
        curr_start = goal->goals[i - 1];
      }
      curr_goal = goal->goals[i];

      // Transform them into the global frame
      if (!transformPosesToGlobalFrame(curr_start, curr_goal)) {
        throw nav2_core::PlannerTFError("Unable to transform poses to global frame");
      }

      // Get plan from start -> goal
      nav_msgs::msg::Path curr_path = getPlan(curr_start, curr_goal, goal->planner_id);

      if (!validatePath<ActionThroughPoses>(curr_goal, curr_path, goal->planner_id)) {
        throw nav2_core::NoValidPathCouldBeFound(goal->planner_id + " generated a empty path");
      }

      // Concatenate paths together
      concat_path.poses.insert(
          concat_path.poses.end(), curr_path.poses.begin(), curr_path.poses.end());
      concat_path.header = curr_path.header;
    }

    // Publish the plan for visualization purposes
    result->path = concat_path;
    publishPlan(result->path);

    auto cycle_duration = steady_clock_.now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
          get_logger(),
          "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
          1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_poses_->succeeded_current(result);
  } catch (nav2_core::InvalidPlanner &ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionToPoseGoal::INVALID_PLANNER;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::StartOccupied &ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesGoal::START_OCCUPIED;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::GoalOccupied &ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesGoal::GOAL_OCCUPIED;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::NoValidPathCouldBeFound &ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesGoal::NO_VALID_PATH;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::PlannerTimedOut &ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesGoal::TIMEOUT;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::StartOutsideMapBounds &ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesGoal::START_OUTSIDE_MAP;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::GoalOutsideMapBounds &ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesGoal::GOAL_OUTSIDE_MAP;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::PlannerTFError &ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesGoal::TF_ERROR;
    action_server_poses_->terminate_current(result);
  } catch (nav2_core::NoViapointsGiven &ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesGoal::NO_VIAPOINTS_GIVEN;
    action_server_poses_->terminate_current(result);
  } catch (std::exception &ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesGoal::UNKNOWN;
    action_server_poses_->terminate_current(result);
  }
}

void PlannerServer::computePlan() {
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = steady_clock_.now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_pose_->get_current_goal();
  auto result = std::make_shared<ActionToPose::Result>();

  geometry_msgs::msg::PoseStamped start;

  try {
    if (isServerInactive(action_server_pose_) || isCancelRequested(action_server_pose_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_pose_, goal);

    // Use start pose if provided otherwise use current robot pose
    if (!getStartPose<ActionToPose>(goal, start)) {
      throw nav2_core::PlannerTFError("Unable to get start pose");
    }

    // Transform them into the global frame
    geometry_msgs::msg::PoseStamped goal_pose = goal->goal;
    if (!transformPosesToGlobalFrame(start, goal_pose)) {
      throw nav2_core::PlannerTFError("Unable to transform poses to global frame");
    }

    result->path = getPlan(start, goal_pose, goal->planner_id);

    if (!validatePath<ActionThroughPoses>(goal_pose, result->path, goal->planner_id)) {
      throw nav2_core::NoValidPathCouldBeFound(goal->planner_id + " generated a empty path");
    }

    // Publish the plan for visualization purposes
    publishPlan(result->path);

    auto cycle_duration = steady_clock_.now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
          get_logger(),
          "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
          1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }
    action_server_pose_->succeeded_current(result);
  } catch (nav2_core::InvalidPlanner &ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex);
    result->error_code = ActionToPoseGoal::INVALID_PLANNER;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::StartOccupied &ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex);
    result->error_code = ActionToPoseGoal::START_OCCUPIED;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::GoalOccupied &ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex);
    result->error_code = ActionToPoseGoal::GOAL_OCCUPIED;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::NoValidPathCouldBeFound &ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex);
    result->error_code = ActionToPoseGoal::NO_VALID_PATH;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::PlannerTimedOut &ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex);
    result->error_code = ActionToPoseGoal::TIMEOUT;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::StartOutsideMapBounds &ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex);
    result->error_code = ActionToPoseGoal::START_OUTSIDE_MAP;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::GoalOutsideMapBounds &ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex);
    result->error_code = ActionToPoseGoal::GOAL_OUTSIDE_MAP;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::PlannerTFError &ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex);
    result->error_code = ActionToPoseGoal::TF_ERROR;
    action_server_pose_->terminate_current(result);
  } catch (std::exception &ex) {
    exceptionWarning(start, goal->goal, goal->planner_id, ex);
    result->error_code = ActionToPoseGoal::UNKNOWN;
    action_server_pose_->terminate_current(result);
  }
}

nav_msgs::msg::Path PlannerServer::getPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal,
    const std::string &planner_id) {
  RCLCPP_DEBUG(
      get_logger(),
      "Attempting to a find path from (%.2f, %.2f) to "
      "(%.2f, %.2f).",
      start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

  if (planners_.find(planner_id) != planners_.end()) {
    return planners_[planner_id]->createPlan(start, goal);
  } else {
    if (planners_.size() == 1 && planner_id.empty()) {
      RCLCPP_WARN_ONCE(
          get_logger(),
          "No planners specified in action call. "
          "Server will use only plugin %s in server."
          " This warning will appear once.",
          planner_ids_concat_.c_str());
      return planners_[planners_.begin()->first]->createPlan(start, goal);
    } else {
      RCLCPP_ERROR(
          get_logger(),
          "planner %s is not a valid planner. "
          "Planner names are: %s",
          planner_id.c_str(), planner_ids_concat_.c_str());
      throw nav2_core::InvalidPlanner("Planner id " + planner_id + " is invalid");
    }
  }

  return nav_msgs::msg::Path();
}

void PlannerServer::publishPlan(const nav_msgs::msg::Path &path) {
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  if (plan_publisher_->is_activated() && plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(std::move(msg));
  }
}

void PlannerServer::isPathValid(
    const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
    std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response) {
  response->is_valid = true;

  if (request->path.poses.empty()) {
    response->is_valid = false;
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  unsigned int closest_point_index = 0;
  if (costmap_ros_->getRobotPose(current_pose)) {
    float current_distance = std::numeric_limits<float>::max();
    float closest_distance = current_distance;
    geometry_msgs::msg::Point current_point = current_pose.pose.position;
    for (unsigned int i = 0; i < request->path.poses.size(); ++i) {
      geometry_msgs::msg::Point path_point = request->path.poses[i].pose.position;

      current_distance = nav2_util::geometry_utils::euclidean_distance(current_point, path_point);

      if (current_distance < closest_distance) {
        closest_point_index = i;
        closest_distance = current_distance;
      }
    }

    /**
     * The lethal check starts at the closest point to avoid points that have already been passed
     * and may have become occupied
     */
    unsigned int mx = 0;
    unsigned int my = 0;
    for (unsigned int i = closest_point_index; i < request->path.poses.size(); ++i) {
      costmap_->worldToMap(
          request->path.poses[i].pose.position.x, request->path.poses[i].pose.position.y, mx, my);
      unsigned int cost = costmap_->getCost(mx, my);

      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
          cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        response->is_valid = false;
      }
    }
  }
}

rcl_interfaces::msg::SetParametersResult PlannerServer::dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters) {
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto &type = parameter.get_type();
    const auto &name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "expected_planner_frequency") {
        if (parameter.as_double() > 0) {
          max_planner_duration_ = 1 / parameter.as_double();
        } else {
          RCLCPP_WARN(
              get_logger(),
              "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
              " than 0.0 to turn on duration overrrun warning messages",
              parameter.as_double());
          max_planner_duration_ = 0.0;
        }
      }
    }
  }

  result.successful = true;
  return result;
}

void PlannerServer::exceptionWarning(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal,
    const std::string &planner_id,
    const std::exception &ex) {
  RCLCPP_WARN(
      get_logger(), "%s plugin failed to plan from (%.2f, %.2f) to (%0.2f, %.2f): \"%s\"",
      planner_id.c_str(), start.pose.position.x, start.pose.position.y, goal.pose.position.x,
      goal.pose.position.y, ex.what());
}

}  // namespace nav2_planner

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_planner::PlannerServer)
