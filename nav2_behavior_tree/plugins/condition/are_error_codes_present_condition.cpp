// Copyright (c) 2023 Joshua Wallace
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

#include "nav2_behavior_tree/plugins/condition/are_error_codes_present_condition.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
// 注册节点 (Register nodes)
BT_REGISTER_NODES(factory)
{
  // 使用工厂对象注册自定义节点类型，这里我们注册了名为 "AreErrorCodesPresent" 的节点 (Use the factory object to register the custom node type, here we register a node called "AreErrorCodesPresent")
  factory.registerNodeType<nav2_behavior_tree::AreErrorCodesPresent>("AreErrorCodesPresent");
}