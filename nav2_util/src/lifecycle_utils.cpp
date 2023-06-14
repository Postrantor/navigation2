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

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_util/lifecycle_service_client.hpp"

using lifecycle_msgs::msg::Transition;
using std::string;

namespace nav2_util {

/*
  其中包含了启动生命周期节点和重置生命周期节点两个函数，分别用于启动和重置一个或多个生命周期节点。

  启动生命周期节点需要使用 LifecycleServiceClient 对象，并执行两个状态转换操作：TRANSITION_CONFIGURE
  和 TRANSITION_ACTIVATE。由于服务调用可能会 hang，因此使用 RETRY 宏定义对函数调用进行重试，最多重试
  retries 次。

  重置生命周期节点同样需要使用 LifecycleServiceClient
  对象，并执行两个状态转换操作：TRANSITION_DEACTIVATE 和 TRANSITION_CLEANUP。也需要使用 RETRY
  宏定义对函数调用进行重试，最多重试 retries 次。

  startup_lifecycle_nodes 和 reset_lifecycle_nodes
  函数分别用于启动和重置多个生命周期节点，对于每个节点，都会调用相应的单个节点启动或重置函数。
*/

/**
 * @brief 宏定义，用于重试函数调用
 * @param fn 需要重试的函数
 * @param retries 重试次数
 * @details 如果函数调用抛出 std::runtime_error 异常，则会进行重试，最多重试 retries 次
 */
#define RETRY(fn, retries)               \
  {                                      \
    int count = 0;                       \
    while (true) {                       \
      try {                              \
        fn;                              \
        break;                           \
      } catch (std::runtime_error & e) { \
        ++count;                         \
        if (count > (retries)) {         \
          throw e;                       \
        }                                \
      }                                  \
    }                                    \
  }

/**
 * @brief 启动生命周期节点
 * @param node_name 节点名称
 * @param service_call_timeout 服务调用超时时间
 * @param retries 重试次数
 * @details 使用 LifecycleServiceClient
 * 对象启动生命周期节点，需要执行两个状态转换操作：TRANSITION_CONFIGURE 和 TRANSITION_ACTIVATE
 *         由于服务调用可能会 hang，因此使用 RETRY 宏定义对函数调用进行重试，最多重试 retries 次
 */
static void startupLifecycleNode(
    const std::string& node_name,
    const std::chrono::seconds service_call_timeout,
    const int retries) {
  // manager 里面应该是直接这样调起来的，不是通过封装后的`startupLifecycleNode`?
  // 全局检索了一下，都没找到哪里包含这个作为头文件以及，调用这个函数?
  LifecycleServiceClient sc(node_name);

  // Despite waiting for the service to be available and using reliable transport
  // service calls still frequently hang. To get reliable startup it's necessary
  // to timeout the service call and retry it when that happens.
  RETRY(sc.change_state(Transition::TRANSITION_CONFIGURE, service_call_timeout), retries);
  RETRY(sc.change_state(Transition::TRANSITION_ACTIVATE, service_call_timeout), retries);
}

/**
 * @brief 启动多个生命周期节点
 * @param node_names 节点名称列表
 * @param service_call_timeout 服务调用超时时间
 * @param retries 重试次数
 * @details 对于每个节点，都调用 startupLifecycleNode 函数进行启动
 */
void startup_lifecycle_nodes(
    const std::vector<std::string>& node_names,
    const std::chrono::seconds service_call_timeout,
    const int retries) {
  for (const auto& node_name : node_names) {
    startupLifecycleNode(node_name, service_call_timeout, retries);
  }
}

/**
 * @brief 重置生命周期节点
 * @param node_name 节点名称
 * @param service_call_timeout 服务调用超时时间
 * @param retries 重试次数
 * @details 使用 LifecycleServiceClient
 * 对象重置生命周期节点，需要执行两个状态转换操作：TRANSITION_DEACTIVATE 和 TRANSITION_CLEANUP
 *          由于服务调用可能会 hang，因此使用 RETRY 宏定义对函数调用进行重试，最多重试 retries 次
 */
static void resetLifecycleNode(
    const std::string& node_name,
    const std::chrono::seconds service_call_timeout,
    const int retries) {
  LifecycleServiceClient sc(node_name);

  // Despite waiting for the service to be available and using reliable transport
  // service calls still frequently hang. To get reliable reset it's necessary
  // to timeout the service call and retry it when that happens.
  RETRY(sc.change_state(Transition::TRANSITION_DEACTIVATE, service_call_timeout), retries);
  RETRY(sc.change_state(Transition::TRANSITION_CLEANUP, service_call_timeout), retries);
}

/**
 * @brief 重置多个生命周期节点
 * @param node_names 节点名称列表
 * @param service_call_timeout 服务调用超时时间
 * @param retries 重试次数
 * @details 对于每个节点，都调用 resetLifecycleNode 函数进行重置
 */
void reset_lifecycle_nodes(
    const std::vector<std::string>& node_names,
    const std::chrono::seconds service_call_timeout,
    const int retries) {
  for (const auto& node_name : node_names) {
    resetLifecycleNode(node_name, service_call_timeout, retries);
  }
}

}  // namespace nav2_util
