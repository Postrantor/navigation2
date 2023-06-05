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

#ifndef NAV2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
#define NAV2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/xml_parsing.h"

namespace nav2_behavior_tree {

/**
 * @enum nav2_behavior_tree::BtStatus
 * @brief 一个表示行为树执行状态的枚举类 (An enum class representing BT execution status)
 */
enum class BtStatus { SUCCEEDED, FAILED, CANCELED };

/**
 * @class nav2_behavior_tree::BehaviorTreeEngine
 * @brief 一个用于创建和处理行为树的类 (A class to create and handle behavior trees)
 */
class BehaviorTreeEngine {
public:
  /**
   * @brief nav2_behavior_tree::BehaviorTreeEngine 的构造函数 (A constructor for
   * nav2_behavior_tree::BehaviorTreeEngine)
   * @param plugin_libraries 要加载的 BT 插件库名称向量 (vector of BT plugin library names to load)
   */
  explicit BehaviorTreeEngine(const std::vector<std::string>& plugin_libraries);
  virtual ~BehaviorTreeEngine() {
  }  // 析构函数，虚拟以便子类可以覆盖 (Destructor, virtual so subclasses can override)

  /**
   * @brief 以特定频率执行 BT 的函数 (Function to execute a BT at a specific rate)
   * @param tree 要执行的 BT (BT to execute)
   * @param onLoop BT 执行每次迭代时要执行的函数 (Function to execute on each iteration of BT
   * execution)
   * @param cancelRequested 在 BT 执行期间检查是否请求取消的函数 (Function to check if cancel was
   * requested during BT execution)
   * @param loopTimeout 每次迭代 BT 执行的时间周期 (Time period for each iteration of BT execution)
   * @return nav2_behavior_tree::BtStatus BT 执行状态 (Status of BT execution)
   */
  BtStatus run(
      BT::Tree* tree,
      std::function<void()> onLoop,
      std::function<bool()> cancelRequested,
      std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

  /**
   * @brief 从 XML 字符串创建 BT 的函数 (Function to create a BT from a XML string)
   * @param xml_string 表示 BT 的 XML 字符串 (XML string representing BT)
   * @param blackboard BT 的黑板 (Blackboard for BT)
   * @return BT::Tree 创建的行为树 (Created behavior tree)
   */
  BT::Tree createTreeFromText(const std::string& xml_string, BT::Blackboard::Ptr blackboard);

  /**
   * @brief 从 XML 文件创建 BT 的函数 (Function to create a BT from an XML file)
   * @param file_path BT XML 文件路径 (Path to BT XML file)
   * @param blackboard BT 的黑板 (Blackboard for BT)
   * @return BT::Tree 创建的行为树 (Created behavior tree)
   */
  BT::Tree createTreeFromFile(const std::string& file_path, BT::Blackboard::Ptr blackboard);

  /**
   * @brief 显式重置所有 BT 节点到初始状态的函数 (Function to explicitly reset all BT nodes to
   * initial state)
   * @param root_node 指向 BT 根节点的指针 (Pointer to BT root node)
   */
  void haltAllActions(BT::TreeNode* root_node);

protected:
  // 将用于动态构造行为树的工厂 (The factory that will be used to dynamically construct the behavior
  // tree)
  BT::BehaviorTreeFactory factory_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
