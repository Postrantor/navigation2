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

#ifndef NAV2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
#define NAV2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_

#include <set>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/time.hpp"

namespace BT {

// 下面的模板是在使用这些类型作为参数时所需的，用于将 BT XML 文件中的字符串解析成相应的数据类型。
// The following templates are required when using these types as parameters,
// they parse the strings in the BT XML files into their corresponding data type.

/**
 * @brief 将 XML 字符串解析为 geometry_msgs::msg::Point
 * @brief Parse XML string to geometry_msgs::msg::Point
 * @param key XML 字符串
 * @param key XML string
 * @return geometry_msgs::msg::Point
 */
template <>
inline geometry_msgs::msg::Point convertFromString(const StringView key) {
  // 三个实数，用分号隔开
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3) {
    // 如果字段数量不正确，则抛出异常
    // throw an exception if the number of fields is incorrect
    throw std::runtime_error("invalid number of fields for point attribute)");
  } else {
    geometry_msgs::msg::Point position;
    // 解析并设置 x 坐标
    // parse and set x coordinate
    position.x = BT::convertFromString<double>(parts[0]);
    // 解析并设置 y 坐标
    // parse and set y coordinate
    position.y = BT::convertFromString<double>(parts[1]);
    // 解析并设置 z 坐标
    // parse and set z coordinate
    position.z = BT::convertFromString<double>(parts[2]);
    return position;
  }
}

/**
 * @brief 将 XML 字符串解析为 geometry_msgs::msg::Quaternion
 * @brief Parse XML string to geometry_msgs::msg::Quaternion
 * @param key XML 字符串
 * @param key XML string
 * @return geometry_msgs::msg::Quaternion
 */
template <>
inline geometry_msgs::msg::Quaternion convertFromString(const StringView key) {
  // 四个实数，用分号隔开
  // four real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 4) {
    // 如果字段数量不正确，则抛出异常
    // throw an exception if the number of fields is incorrect
    throw std::runtime_error("invalid number of fields for orientation attribute)");
  } else {
    geometry_msgs::msg::Quaternion orientation;
    // 解析并设置 x 分量
    // parse and set x component
    orientation.x = BT::convertFromString<double>(parts[0]);
    // 解析并设置 y 分量
    // parse and set y component
    orientation.y = BT::convertFromString<double>(parts[1]);
    // 解析并设置 z 分量
    // parse and set z component
    orientation.z = BT::convertFromString<double>(parts[2]);
    // 解析并设置 w 分量
    // parse and set w component
    orientation.w = BT::convertFromString<double>(parts[3]);
    return orientation;
  }
}

/**
 * @brief 将 XML 字符串解析为 geometry_msgs::msg::PoseStamped (Parse XML string to
 * geometry_msgs::msg::PoseStamped)
 * @param key XML 字符串 (XML string)
 * @return geometry_msgs::msg::PoseStamped 对象 (geometry_msgs::msg::PoseStamped object)
 */
template <>
inline geometry_msgs::msg::PoseStamped convertFromString(const StringView key) {
  // 以分号分隔的 7 个实数 (7 real numbers separated by semicolons)
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 9) {
    throw std::runtime_error("invalid number of fields for PoseStamped attribute)");
  } else {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = rclcpp::Time(BT::convertFromString<int64_t>(parts[0]));
    pose_stamped.header.frame_id = BT::convertFromString<std::string>(parts[1]);
    pose_stamped.pose.position.x = BT::convertFromString<double>(parts[2]);
    pose_stamped.pose.position.y = BT::convertFromString<double>(parts[3]);
    pose_stamped.pose.position.z = BT::convertFromString<double>(parts[4]);
    pose_stamped.pose.orientation.x = BT::convertFromString<double>(parts[5]);
    pose_stamped.pose.orientation.y = BT::convertFromString<double>(parts[6]);
    pose_stamped.pose.orientation.z = BT::convertFromString<double>(parts[7]);
    pose_stamped.pose.orientation.w = BT::convertFromString<double>(parts[8]);
    return pose_stamped;
  }
}

/**
 * @brief 将 XML 字符串解析为 std::chrono::milliseconds (Parse XML string to
 * std::chrono::milliseconds)
 * @param key XML 字符串 (XML string)
 * @return std::chrono::milliseconds 对象 (std::chrono::milliseconds object)
 */
template <>
inline std::chrono::milliseconds convertFromString<std::chrono::milliseconds>(
    const StringView key) {
  return std::chrono::milliseconds(std::stoul(key.data()));
}

/**
 * @brief 将 XML 字符串解析为 std::set<int> (Parse XML string to std::set<int>)
 * @param key XML 字符串 (XML string)
 * @return std::set<int> 对象 (std::set<int> object)
 */
template <>
inline std::set<int> convertFromString(StringView key) {
  // 以分号分隔的实数 (Real numbers separated by semicolons)
  auto parts = splitString(key, ';');

  std::set<int> set;
  for (const auto part : parts) {
    set.insert(convertFromString<int>(part));
  }
  return set;
}

}  // namespace BT

#endif  // NAV2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
