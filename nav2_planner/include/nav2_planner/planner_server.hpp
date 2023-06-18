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

#ifndef NAV2_PLANNER__PLANNER_SERVER_HPP_
#define NAV2_PLANNER__PLANNER_SERVER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_planner {

// clang-format off
/*
  这段代码定义了一个名为PlannerServer的类，该类继承自nav2_util::LifecycleNode。它是一个动作服务器，实现了行为树的ComputePathToPose接口，并托管不同算法的各种插件来计算路径。

  其中，`explicit PlannerServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())`是构造函数，
  `~PlannerServer()`是析构函数。
  PlannerMap是一个类型别名，它定义了一个无序映射表，将字符串映射到全局路径规划器的指针。

  getPlan()方法接受起始位姿、目标请求和路径规划器ID作为参数，并返回一条路径。
*/
// clang-format on

/**
 * @class nav2_planner::PlannerServer
 * @brief 一个动作服务器，实现了行为树的ComputePathToPose接口，并托管不同算法的各种插件来计算路径。
 */
class PlannerServer : public nav2_util::LifecycleNode {
public:
  /**
   * @brief nav2_planner::PlannerServer的构造函数
   * @param options 控制节点创建的其他选项。
   */
  explicit PlannerServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  /**
   * @brief nav2_planner::PlannerServer的析构函数
   */
  ~PlannerServer();

  using PlannerMap = std::unordered_map<std::string, nav2_core::GlobalPlanner::Ptr>;

  /**
   * @brief 从所需的插件中获取路径规划
   * @param start 起始位姿
   * @param goal 目标请求
   * @return Path
   */
  nav_msgs::msg::Path getPlan(
      const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal,
      const std::string& planner_id);

protected:
  /*
    定义了五个函数，分别是 on_configure、on_activate、on_deactivate、on_cleanup 和
    on_shutdown。这些函数都是虚函数，需要在派生类中进行实现。

    这些函数都接收一个 rclcpp_lifecycle::State 类型的参数 state，表示当前节点的状态。
    这些函数的返回值类型为 nav2_util::CallbackReturn，表示回调函数的返回值。

    这些函数的具体功能如下：
    - on_configure：配置成员变量并初始化规划器。
    - on_activate：激活成员变量。
    - on_deactivate：停用成员变量。
    - on_cleanup：重置成员变量。
    - on_shutdown：在关闭状态下调用。
  */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  using ActionToPose = nav2_msgs::action::ComputePathToPose;
  using ActionToPoseGoal = ActionToPose::Goal;
  using ActionThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  using ActionThroughPosesGoal = ActionThroughPoses::Goal;
  using ActionServerToPose = nav2_util::SimpleActionServer<ActionToPose>;
  using ActionServerThroughPoses = nav2_util::SimpleActionServer<ActionThroughPoses>;

  // clang-format off
  /*
    其中包含了5个函数模板，分别用于
    - 检查操作服务器是否有效/活动、
    - 检查操作服务器是否有取消请求待处理、
    - 等待代价地图使用更新的传感器数据或在清除恢复后重新填充、
    - 检查操作服务器是否有预占请求，并用新的预占目标替换目标、
    - 从代价地图或消息中获取起始姿态，如果有效。

    其中，
    - 函数模板isServerInactive()和isCancelRequested()用于检查操作服务器是否有效/活动以及是否有取消请求待处理，返回成功或失败；
    - waitForCostmap()函数用于等待代价地图使用更新的传感器数据或在清除恢复后重新填充，阻塞直到为true，没有超时；
    - getPreemptedGoalIfRequested()函数用于检查操作服务器是否有预占请求，并用新的预占目标替换目标；
    - getStartPose()函数从代价地图或消息中获取起始姿态，如果有效，返回bool值表示是否成功找到有效的起始姿态。
  */
  // clang-format on

  /**
   * @brief 检查操作服务器是否有效/活动
   * @param action_server 要测试的操作服务器
   * @return 成功或失败
   */
  template <typename T>
  bool isServerInactive(std::unique_ptr<nav2_util::SimpleActionServer<T>>& action_server);

  /**
   * @brief 检查操作服务器是否有取消请求待处理
   * @param action_server 要测试的操作服务器
   * @return 成功或失败
   */
  template <typename T>
  bool isCancelRequested(std::unique_ptr<nav2_util::SimpleActionServer<T>>& action_server);

  /**
   * @brief 等待代价地图使用更新的传感器数据或在清除恢复后重新填充。阻塞直到为true，没有超时。
   */
  void waitForCostmap();

  /**
   * @brief 检查操作服务器是否有预占请求，并用新的预占目标替换目标。
   * @param action_server 如果需要，则从中获取更新的目标的操作服务器
   * @param goal 要覆盖的目标
   */
  template <typename T>
  void getPreemptedGoalIfRequested(
      std::unique_ptr<nav2_util::SimpleActionServer<T>>& action_server,
      typename std::shared_ptr<const typename T::Goal> goal);

  /**
   * @brief 从代价地图或消息中获取起始姿态，如果有效
   * @param action_server 如果需要，则终止操作服务器
   * @param goal 要从中查找起点的目标
   * @param start 要使用的起始姿态
   * @return bool 是否成功找到有效的起始姿态
   */
  template <typename T>
  bool getStartPose(
      typename std::shared_ptr<const typename T::Goal> goal,
      geometry_msgs::msg::PoseStamped& start);

  /**
   * @brief 将起点和终点的姿态转换为costmap全局坐标系，以便路径规划插件使用
   * @param start 要转换的起始姿态
   * @param goal 要转换的目标姿态
   * @return bool 如果成功转换姿态则返回true
   */
  bool transformPosesToGlobalFrame(
      geometry_msgs::msg::PoseStamped& curr_start, geometry_msgs::msg::PoseStamped& curr_goal);

  /**
   * @brief 验证路径是否包含有意义的路径
   * @param action_server 如果需要，则终止操作服务器
   * @param goal 当前目标
   * @param path 当前路径
   * @param planner_id 用于生成路径的规划器ID
   * @return bool 如果路径有效则返回true
   */
  template <typename T>
  bool validatePath(
      const geometry_msgs::msg::PoseStamped& curr_goal,
      const nav_msgs::msg::Path& path,
      const std::string& planner_id);

  /**
   * @brief 调用规划器获取路径的操作服务器回调函数 ComputePathToPose
   */
  void computePlan();

  /**
   * @brief 调用规划器获取路径的操作服务器回调函数 ComputePathThroughPoses
   */
  void computePlanThroughPoses();

  /**
   * @brief 确定路径是否仍然有效的服务回调函数
   * @param request 服务请求
   * @param response 服务响应
   */
  void isPathValid(
      const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
      std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response);

  /**
   * @brief Publish a path for visualization purposes
   * @param path Reference to Global Path
   */
  void publishPlan(const nav_msgs::msg::Path& path);

  void exceptionWarning(
      const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal,
      const std::string& planner_id,
      const std::exception& ex);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
      std::vector<rclcpp::Parameter> parameters);

  // Our action server implements the ComputePathToPose action
  std::unique_ptr<ActionServerToPose> action_server_pose_;
  std::unique_ptr<ActionServerThroughPoses> action_server_poses_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  // Planner
  PlannerMap planners_;
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> gp_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> planner_ids_;
  std::vector<std::string> planner_types_;
  double max_planner_duration_;
  std::string planner_ids_concat_;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Global Costmap
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  nav2_costmap_2d::Costmap2D* costmap_;

  // Publishers for the path
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

  // Service to determine if the path is valid
  rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr is_path_valid_service_;
};

}  // namespace nav2_planner

#endif  // NAV2_PLANNER__PLANNER_SERVER_HPP_
