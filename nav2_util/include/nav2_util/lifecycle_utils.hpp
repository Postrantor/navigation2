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

#ifndef NAV2_UTIL__LIFECYCLE_UTILS_HPP_
#define NAV2_UTIL__LIFECYCLE_UTILS_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "nav2_util/string_utils.hpp"

namespace nav2_util {

/* README
  这个文件主要是提供了 startup_lifecycle_nodes()\reset_lifecycle_nodes() 两个方法
  接收的参数应该是从配置文件中获取的，传入一组节点名称并逐个转换到指定状态

  这样可以保证节点的启动顺序？
    应该是通过按顺序给节点发送启动转换的指令，但是不是保证一个启动之后，另一个启动
    这个应该是级联启动的操作，需要参考 BICA 的实现。
*/

/// Transition the given lifecycle nodes to the ACTIVATED state in order
/** At this time, service calls frequently hang for unknown reasons. The only
 *  way to combat that is to timeout the service call and retry it. To use this
 *  function, estimate how long your nodes should take to at each transition and
 *  set your timeout accordingly.
 * \param[in] node_names A vector of the fully qualified node names to startup.
 * \param[in] service_call_timeout The maximum amount of time to wait for a
 *            service call.
 * \param[in] retries The number of times to try a state transition service call
 */
/**
 * @brief 将给定的生命周期节点按顺序转换为 ACTIVATED 状态
 * @param[in] node_names 要启动的完全限定节点名称的向量
 * @param[in] service_call_timeout 等待服务调用的最长时间
 * @param[in] retries 尝试状态转换服务调用的次数
 * @details 此时，由于未知原因，服务调用经常会挂起。唯一的方法是超时服务调用并重试它。
 *         为使用此函数，请估计每个转换所需的节点运行时间，并相应地设置超时。
 */
void startup_lifecycle_nodes(
    const std::vector<std::string>& node_names,
    const std::chrono::seconds service_call_timeout = std::chrono::seconds::max(),
    const int retries = 3);

/// Transition the given lifecycle nodes to the ACTIVATED state in order.
/**
 * \param[in] nodes A ':' seperated list of node names. eg. "/node1:/node2"
 */
/**
 * @brief 将给定的生命周期节点按顺序转换为 ACTIVATED 状态
 * @param[in] nodes 节点名称的 ':' 分隔列表。例如："/node1:/node2"
 * @param[in] service_call_timeout 等待服务调用的最长时间
 * @param[in] retries 尝试状态转换服务调用的次数
 */
void startup_lifecycle_nodes(
    const std::string& nodes,
    const std::chrono::seconds service_call_timeout = std::chrono::seconds::max(),
    const int retries = 3) {
  startup_lifecycle_nodes(split(nodes, ':'), service_call_timeout, retries);
}

/// Transition the given lifecycle nodes to the UNCONFIGURED state in order
/** At this time, service calls frequently hang for unknown reasons. The only
 *  way to combat that is to timeout the service call and retry it. To use this
 *  function, estimate how long your nodes should take to at each transition and
 *  set your timeout accordingly.
 * \param[in] node_names A vector of the fully qualified node names to reset.
 * \param[in] service_call_timeout The maximum amount of time to wait for a
 *            service call.
 * \param[in] retries The number of times to try a state transition service call
 */
/**
 * @brief 将给定的生命周期节点按顺序转换为 UNCONFIGURED 状态
 * @param[in] node_names 要重置的完全限定节点名称的向量
 * @param[in] service_call_timeout 等待服务调用的最长时间
 * @param[in] retries 尝试状态转换服务调用的次数
 * @details 此时，由于未知原因，服务调用经常会挂起。唯一的方法是超时服务调用并重试它。
 * 为使用此函数，请估计每个转换所需的节点运行时间，并相应地设置超时。
 */
void reset_lifecycle_nodes(
    const std::vector<std::string>& node_names,
    const std::chrono::seconds service_call_timeout = std::chrono::seconds::max(),
    const int retries = 3);

/// Transition the given lifecycle nodes to the UNCONFIGURED state in order.
/**
 * \param[in] nodes A ':' seperated list of node names. eg. "/node1:/node2"
 */
/**
 * @brief 将给定的生命周期节点按顺序转换为 UNCONFIGURED 状态
 * @param[in] nodes 节点名称的 ':' 分隔列表。例如："/node1:/node2"
 * @param[in] service_call_timeout 等待服务调用的最长时间
 * @param[in] retries 尝试状态转换服务调用的次数
 */
void reset_lifecycle_nodes(
    const std::string& nodes,
    const std::chrono::seconds service_call_timeout = std::chrono::seconds::max(),
    const int retries = 3) {
  reset_lifecycle_nodes(split(nodes, ':'), service_call_timeout, retries);
}

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_UTILS_HPP_
