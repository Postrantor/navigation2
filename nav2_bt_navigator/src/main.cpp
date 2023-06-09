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

#include <memory>                              // 引入头文件 memory

#include "nav2_bt_navigator/bt_navigator.hpp"  // 引入头文件 bt_navigator.hpp
#include "rclcpp/rclcpp.hpp"                   // 引入头文件 rclcpp.hpp

/*
该代码段是一个 ROS 2 项目中 nav2_bt_navigator 组件相关的代码。其中，主函数通过初始化 ROS 2
节点、创建一个 BtNavigator 对象、启动节点并等待退出的方式来实现导航功能。具体实现过程如下：

1. 引入头文件 memory 和 bt_navigator.hpp，以及 rclcpp.hpp。
1. 在主函数中，首先调用 rclcpp::init() 函数初始化 ROS 2 节点。
1. 然后，使用 std::make_shared() 函数创建一个 BtNavigator 对象，并将其赋值给 auto node。
1. 接着，调用 rclcpp::spin() 函数启动节点并等待退出。
1. 最后，调用 rclcpp::shutdown() 函数关闭 ROS 2 节点。
1. 主函数返回值为整型 0。
*/
/**
 * @brief 主函数
 * @param argc 参数个数
 * @param argv 参数列表
 * @return 返回值为整型 0
 * @details 初始化 ROS 2 节点，创建一个 BtNavigator 对象，启动节点并等待退出。
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);                                        // 初始化 ROS 2 节点
  auto node = std::make_shared<nav2_bt_navigator::BtNavigator>();  // 创建 BtNavigator 对象
  rclcpp::spin(node->get_node_base_interface());                   // 启动节点并等待退出
  rclcpp::shutdown();                                              // 关闭 ROS 2 节点

  return 0;                                                        // 返回值为整型 0
}

/*
## 启动节点并等待退出 `rclcpp::spin(node->get_node_base_interface());`

当调用 `rclcpp::spin(node->get_node_base_interface())` 函数时，ROS 2
节点将开始运行，并且会阻塞当前线程直到节点停止。该函数的参数是一个指向
`rclcpp::node_interfaces::NodeBaseInterface`
接口的指针，它提供了节点的基本功能，如获取节点名称、创建发布者和订阅者等。在这里，我们使用
`node->get_node_base_interface()` 获取了 BtNavigator 对象的节点基础接口，并将其作为参数传递给
`rclcpp::spin()` 函数。

    当 ROS 2 节点开始运行时，它会执行以下操作：

    1. 初始化所有已注册的节点。
    2. 创建并启动所有已注册的回调函数。
    3. 等待节点停止信号。
    4. 当收到节点停止信号时，停止所有回调函数并关闭节点。

    因此，调用 `rclcpp::spin()` 函数是启动 ROS 2 节点的关键步骤之一。

## 进一步的对 `node->get_node_base_interface()` 解释一下

`node->get_node_base_interface()` 是一个 ROS 2 C++ API，用于获取节点的基础接口。在 ROS 2
中，**节点是一个独立的进程或线程，负责执行特定的任务**。节点可以发布和订阅话题、提供和使用服务、创建定时器和回调函数等。

`rclcpp::node_interfaces::NodeBaseInterface`
接口提供了一组函数，用于管理节点的基本功能。这些功能包括获取节点名称、创建发布者和订阅者、创建服务和客户端、创建定时器和回调函数等。通过获取节点的基础接口，我们可以使用这些函数来实现节点的各种功能。

例如，在ROS 2中，要创建一个发布者，我们需要使用节点的基础接口来创建一个 `rclcpp::Publisher`
对象。同样地，要创建一个订阅者，我们需要使用节点的基础接口来创建一个 `rclcpp::Subscription` 对象。

因此，`node->get_node_base_interface()` 函数是获取节点基础接口的关键步骤之一，它允许我们使用 ROS 2
C++ API 来管理节点的各种功能。

## 如何通过接口创建发布者和订阅者

其中，`create_publisher()` 函数用于创建一个指定类型的发布者，`create_subscription()` 函数用于创建一个指定类型的订阅者。这两个函数都需要传入一个话题名称和一个 QoS 历史深度参数，以及一些其他的选项参数。

在这两个函数内部，会调用 `rcl_create_publisher()` 和 `rcl_create_subscription()` 函数来创建真正的发布者和订阅者。这两个函数是 ROS 2 C 库中的函数，用于创建发布者和订阅者的实例，并将其注册到 ROS 2 网络中。

最后，`create_publisher()` 和 `create_subscription()` 函数会返回一个 `std::shared_ptr` 类型的对象，该对象包含了指向创建的发布者或订阅者的指针。这个对象可以被用于在程序中引用发布者或订阅者，并进行相应的操作。

因此，通过节点基础接口提供的 `create_publisher()` 和 `create_subscription()` 函数，我们可以方便地创建发布者和订阅者，并在程序中使用它们来进行消息的发布和订阅。
*/
