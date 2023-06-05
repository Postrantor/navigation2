// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Florian Gramss
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

#include "nav2_behavior_tree/behavior_tree_engine.hpp"

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree {

/**
 * @brief 构造函数，用于加载插件库 (Constructor for loading plugin libraries)
 *
 * @param[in] plugin_libraries 插件库名称列表 (List of plugin library names)
 */
BehaviorTreeEngine::BehaviorTreeEngine(const std::vector<std::string>& plugin_libraries) {
  BT::SharedLibrary loader;
  // 遍历插件库名称列表，并注册每个插件库 (Iterate through the plugin library names and register
  // each plugin library)
  for (const auto& p : plugin_libraries) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }
}

/**
 * @brief 运行行为树 (Run the behavior tree)
 *
 * @param[in] tree 行为树指针 (Pointer to the behavior tree)
 * @param[in] onLoop 循环时调用的回调函数 (Callback function called during loop)
 * @param[in] cancelRequested 检查取消请求的回调函数 (Callback function to check for cancellation
 * request)
 * @param[in] loopTimeout 循环超时时间 (Loop timeout duration)
 * @return BtStatus 返回运行状态 (Return the running status)
 */
BtStatus BehaviorTreeEngine::run(
    BT::Tree* tree,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout) {
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // 在 ROS 正常运行且节点状态为 RUNNING 时循环 (Loop while ROS is running and node status is
  // RUNNING)
  try {
    while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
      // 如果收到取消请求，停止树并返回 CANCELED 状态 (If cancellation is requested, halt the tree
      // and return CANCELED status)
      if (cancelRequested()) {
        tree->rootNode()->halt();
        return BtStatus::CANCELED;
      }

      // 执行行为树的根节点 (Execute the root node of the behavior tree)
      result = tree->tickRoot();

      // 调用循环回调函数 (Call the loop callback function)
      onLoop();

      // 暂停循环一段时间 (Pause the loop for a duration)
      loopRate.sleep();
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(
        rclcpp::get_logger("BehaviorTreeEngine"),
        "Behavior tree threw exception: %s. Exiting with failure.", ex.what());
    return BtStatus::FAILED;
  }

  // 根据节点状态返回运行结果 (Return the running result based on the node status)
  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

/**
 * @brief 从文本创建行为树 (Create behavior tree from text)
 *
 * @param[in] xml_string XML 文本 (XML text)
 * @param[in] blackboard 黑板指针 (Pointer to the blackboard)
 * @return BT::Tree 返回构建好的行为树 (Return the constructed behavior tree)
 */
BT::Tree BehaviorTreeEngine::createTreeFromText(
    const std::string& xml_string, BT::Blackboard::Ptr blackboard) {
  return factory_.createTreeFromText(xml_string, blackboard);
}

/**
 * @brief 从文件创建行为树 (Create behavior tree from file)
 *
 * @param[in] file_path 文件路径 (File path)
 * @param[in] blackboard 黑板指针 (Pointer to the blackboard)
 * @return BT::Tree 返回构建好的行为树 (Return the constructed behavior tree)
 */
BT::Tree BehaviorTreeEngine::createTreeFromFile(
    const std::string& file_path, BT::Blackboard::Ptr blackboard) {
  return factory_.createTreeFromFile(file_path, blackboard);
}

/**
 * @brief 停止所有行为节点，重置行为树状态 (Halt all action nodes and reset the behavior tree state)
 *
 * @param[in] root_node 行为树的根节点指针 (Pointer to the root node of the behavior tree)
 */
void BehaviorTreeEngine::haltAllActions(BT::TreeNode* root_node) {
  if (!root_node) {
    return;
  }

  // 停止信号应该在整个树中传播 (The halt signal should propagate through the entire tree)
  root_node->halt();

  // but, just in case...
  auto visitor = [](BT::TreeNode* node) {
    if (node->status() == BT::NodeStatus::RUNNING) {
      node->halt();
    }
  };
  BT::applyRecursiveVisitor(root_node, visitor);
}

}  // namespace nav2_behavior_tree
