---
translate by baidu@2023-06-03 00:47:21
...

# BT Navigator

The BT Navigator (Behavior Tree Navigator) module implements the NavigateToPose and NavigateThroughPoses task interfaces. It is a [Behavior Tree](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/docs/BT_basics.md)-based implementation of navigation that is intended to allow for flexibility in the navigation task and provide a way to easily specify complex robot behaviors.

> BT 导航器（行为树导航器）模块实现了 NavigateToPose 和 NavigateThroughPoses 任务界面。这是一个[行为树](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/docs/BT_basics.md)-基于导航的实现，旨在允许导航任务的灵活性，并提供一种轻松**指定复杂机器人行为**的方法。

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-bt-navigator.html) for additional parameter descriptions, as well as the [Nav2 Behavior Tree Explanation](https://navigation.ros.org/behavior_trees/index.html) pages explaining more context on the default behavior trees and examples provided in this package.

> 请参阅其[配置指南页面]有关其他参数描述，以及[Nav2 行为树解释]页面解释了有关默认行为树的更多上下文以及此包中提供的示例。

## Overview

The BT Navigator receives a goal pose and navigates the robot to the specified destination(s). To do so, the module reads an XML description of the Behavior Tree from a file, as specified by a Node parameter, and passes that to a generic [BehaviorTreeEngine class](../nav2_behavior_tree/include/nav2_behavior_tree/behavior_tree_engine.hpp) which uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) to dynamically create and execute the BT. The BT XML can also be specified on a per-task basis so that your robot may have many different types of navigation or autonomy behaviors on a per-task basis.

> BT 导航器接收目标姿势并将机器人导航到指定的目的地。为此，模块从文件中读取由 Node 参数指定的行为树的 XML 描述，并将其传递给使用[Behavior-Tree.CPP 库]的通用[Behavior 树引擎类]以动态创建和执行BT。**BT XML 也可以在每个任务的基础上指定，这样您的机器人在每个任务上可能具有许多不同类型的导航或自主行为**。
