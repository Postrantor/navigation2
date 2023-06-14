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

#include "nav2_util/node_utils.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <string>

using std::isalnum;
using std::replace_if;
using std::string;
using std::to_string;
using std::chrono::high_resolution_clock;

namespace nav2_util {

/*
  这段代码主要是用于生成内部节点的名称和实例。其中，
  - sanitize_node_name() 函数将潜在的节点名称中非字母数字字符替换为下划线，并返回处理后的节点名称；
  - add_namespaces() 函数将子命名空间添加到顶层命名空间中，并返回完整的命名空间；
  - time_to_string() 函数将当前时间转换为指定长度的字符串；
  - generate_internal_node_name() 函数使用前缀和当前时间生成节点名称；generate_internal_node()
  函数创建内部节点。

  在 generate_internal_node() 函数中，首先通过 rclcpp::NodeOptions()
  创建一个选项对象，然后设置不启动参数服务和参数事件发布器，并添加命令行参数。
  最后，调用 rclcpp::Node::make_shared() 函数创建内部节点并返回其指针。
*/

/**
 * @brief 将潜在的节点名称中非字母数字字符替换为下划线，并返回处理后的节点名称
 * @param potential_node_name 潜在的节点名称
 * @return 处理后的节点名称
 */
string sanitize_node_name(const string& potential_node_name) {
  string node_name(potential_node_name);
  // 如果不是字母数字，则将 `node_name` 中的字符替换为 '_'
  replace_if(
      begin(node_name), end(node_name), [](auto c) { return !isalnum(c); }, '_');
  return node_name;
}

/**
 * @brief 将子命名空间添加到顶层命名空间中，并返回完整的命名空间
 * @param top_ns 顶层命名空间
 * @param sub_ns 子命名空间
 * @return 完整的命名空间
 */
string add_namespaces(const string& top_ns, const string& sub_ns) {
  if (!top_ns.empty() && top_ns.back() == '/') {
    if (top_ns.front() == '/') {
      return top_ns + sub_ns;
    } else {
      return "/" + top_ns + sub_ns;
    }
  }

  return top_ns + "/" + sub_ns;
}

/**
 * @brief 将当前时间转换为指定长度的字符串
 * @param len 字符串长度
 * @return 转换后的字符串
 */
std::string time_to_string(size_t len) {
  string output(len, '0');                                // 使用 '0' 填充字符串
  auto timepoint = high_resolution_clock::now();          // 获取当前时间点
  auto timecount = timepoint.time_since_epoch().count();  // 获取时间点的计数值
  auto timestring = to_string(timecount);                 // 将计数值转换为字符串
  if (timestring.length() >= len) {
    // 如果 `timestring` 的长度大于等于 `len`，则将其放在 `output` 的末尾
    output.replace(0, len, timestring, timestring.length() - len, len);
  } else {
    // 如果 `output` 的长度大于 `timestring`，则将 `timestring` 的末尾复制到 `output` 中
    output.replace(
        len - timestring.length(), timestring.length(), timestring, 0, timestring.length());
  }
  return output;
}

/**
 * @brief 生成内部节点名称
 * @param prefix 节点名称前缀
 * @return 内部节点名称
 */
std::string generate_internal_node_name(const std::string& prefix) {
  return sanitize_node_name(prefix) + "_" + time_to_string(8);  // 使用前缀和当前时间生成节点名称
}

/**
 * @brief 生成内部节点
 * @param prefix 节点名称前缀
 * @return 内部节点指针
 */
rclcpp::Node::SharedPtr generate_internal_node(const std::string& prefix) {
  auto options =
      rclcpp::NodeOptions()
          .start_parameter_services(false)         // 不启动参数服务
          .start_parameter_event_publisher(false)  // 不启动参数事件发布器
                                                   // 添加命令行参数
          .arguments({"--ros-args", "-r", "__node:=" + generate_internal_node_name(prefix), "--"});
  return rclcpp::Node::make_shared("_", options);  // 创建内部节点
}

}  // namespace nav2_util
