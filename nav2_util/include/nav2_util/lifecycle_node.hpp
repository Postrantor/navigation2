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

#ifndef NAV2_UTIL__LIFECYCLE_NODE_HPP_
#define NAV2_UTIL__LIFECYCLE_NODE_HPP_

#include <memory>
#include <string>
#include <thread>

#include "bond/msg/constants.hpp"
#include "bondcpp/bond.hpp"
#include "nav2_util/node_thread.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_util {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class nav2_util::LifecycleNode
 * @brief 一个生命周期节点的包装器，用于实现常见的 Nav2 需求，例如操作参数
 */
class LifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
  /**
   * @brief 生命周期节点构造函数
   * @param node_name 节点名称
   * @param namespace 命名空间，如果有的话
   * @param options 节点选项
   */
  LifecycleNode(
      const std::string& node_name,
      const std::string& ns = "",
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  virtual ~LifecycleNode();

  /**
   * @brief floating_point_range 和 integer_range，分别表示浮点数范围和整数范围。
   */
  typedef struct {
    double from_value;  // 浮点数范围的起始值
    double to_value;    // 浮点数范围的结束值
    double step;        // 浮点数范围的步长
  } floating_point_range;
  typedef struct {
    int from_value;  // 整数范围的起始值
    int to_value;    // 整数范围的结束值
    int step;        // 整数范围的步长
  } integer_range;

  /**
   * @brief 声明一个没有整数或浮点数范围约束的参数
   * @param node_name 参数名称
   * @param default_value 默认节点添加的值
   * @param description 节点描述
   * @param additional_constraints 对参数的任何其他约束条件进行列举
   * @param read_only 是否将此参数视为只读
   * @details
   *     然后定义了一个函数 add_parameter()，用于声明一个没有整数或浮点数范围约束的参数。
   *     该函数接受五个参数：参数名称、默认节点添加的值、节点描述、对参数的任何其他约束条件进行列举和是否将此参数视为只读。
   *     在函数内部，创建了一个参数描述符，并设置了参数名称、参数描述、参数的其他约束条件和参数是否只读。最后，通过调用
   *     declare_parameter 函数来声明参数。
   */
  void add_parameter(
      const std::string& name,
      const rclcpp::ParameterValue& default_value,
      const std::string& description = "",
      const std::string& additional_constraints = "",
      bool read_only = false) {
    // 创建参数描述符
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.name = name;                                      // 设置参数名称
    descriptor.description = description;                        // 设置参数描述
    descriptor.additional_constraints = additional_constraints;  // 设置参数的其他约束条件
    descriptor.read_only = read_only;                            // 设置参数是否只读
    // 声明参数
    declare_parameter(descriptor.name, default_value, descriptor);
  }

  /**
   * @brief 声明一个具有浮点数范围约束的参数
   * @param node_name 参数名称
   * @param default_value 默认值
   * @param fp_range 浮点数范围
   * @param description 参数描述
   * @param additional_constraints 参数的其他限制条件
   * @param read_only 是否只读
   * @details 将参数描述符中的各项赋值，包括参数名称、描述、附加限制条件、是否只读等。
   *          然后调用 declare_parameter 函数声明该参数。
   */
  void add_parameter(
      const std::string& name,
      const rclcpp::ParameterValue& default_value,
      const floating_point_range fp_range,
      const std::string& description = "",
      const std::string& additional_constraints = "",
      bool read_only = false) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = fp_range.from_value;
    descriptor.floating_point_range[0].to_value = fp_range.to_value;
    descriptor.floating_point_range[0].step = fp_range.step;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  /**
   * @brief 添加一个具有整数范围约束的参数
   * @param node_name 参数名
   * @param default_value 默认值
   * @param integer_range 整数范围
   * @param description 参数描述
   * @param additional_constraints 参数的其他限制条件
   * @param read_only 是否只读
   * @details 将参数添加到节点中，并设置参数的描述符，包括名称、描述、附加限制和是否只读等信息。
   */
  void add_parameter(
      const std::string& name,
      const rclcpp::ParameterValue& default_value,
      const integer_range int_range,
      const std::string& description = "",
      const std::string& additional_constraints = "",
      bool read_only = false) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = int_range.from_value;
    descriptor.integer_range[0].to_value = int_range.to_value;
    descriptor.integer_range[0].step = int_range.step;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  /**
   * @brief 获取此对象的共享指针
   * @details 返回一个指向当前对象的共享指针，该指针可以在多个地方引用同一个对象。
   * [](..\ros\shared_from_this_20230614.md)
   */
  std::shared_ptr<nav2_util::LifecycleNode> shared_from_this() {
    return std::static_pointer_cast<nav2_util::LifecycleNode>(
        rclcpp_lifecycle::LifecycleNode::shared_from_this());
  }

  /**
   * @brief 抽象的 on_error 状态转换回调函数，因为在管理的 ROS2 节点状态机中未实现（2020年）
   * @param state 错误转换之前的状态
   * @return 成功或失败转换到错误状态的返回类型
   */
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_FATAL(
        get_logger(), "Lifecycle node %s does not have error state implemented", get_name());
    return nav2_util::CallbackReturn::SUCCESS;
  }

  /**
   * @brief 在我们的 Context 关闭之前执行 preshutdown 活动。
   *       注意，这与我们的 Context 的关闭序列有关，而不是生命周期节点状态机。
   */
  virtual void on_rcl_preshutdown();

  /*
    - `createBond` 函数：创建与生命周期管理器的 bond 连接。
    - `destroyBond` 函数：销毁与生命周期管理器的 bond 连接。
    这两部分可以通过dds 中的liveliness来替代，这个库的设计初衷是为了兼容底层没有使用dds的中间件
  */
  // 创建与生命周期管理器的 bond 连接
  void createBond();
  // 销毁与生命周期管理器的 bond 连接
  void destroyBond();

protected:
  /**
   * @brief 打印生命周期节点通知
   */
  void printLifecycleNodeNotification();

  /**
   * 为此节点的 rcl Context 注册我们的 preshutdown 回调。
   * 在此节点的 Context 关闭之前，回调会触发。
   * 请注意，这与生命周期状态机没有直接关系。
   */
  void register_rcl_preshutdown_callback();
  std::unique_ptr<rclcpp::PreShutdownCallbackHandle>  //
      rcl_preshutdown_cb_handle_{nullptr};

  /**
   * 运行一些在 rcl preshutdown 和销毁之间共享的常见清理步骤。
   */
  void runCleanups();

  // 用于告知服务器仍在运行的连接
  std::unique_ptr<bond::Bond> bond_{nullptr};
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_NODE_HPP_
