---
tip: translate by baidu@2023-06-06 08:54:19
...

# nav2_behavior_tree

This module is used by the nav2_bt_navigator to implement a ROS2 node that executes navigation Behavior Trees for either navigation or autonomy systems. The nav2_behavior_tree module uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) for the core Behavior Tree processing.

> 该模块由 nav2_bt_navigator 用于实现 ROS2 节点，该节点执行导航或自主系统的导航行为树。nav2_behavior_tree 模块使用[behavior-tree.CPP 库]用于核心行为树处理。

The nav2_behavior_tree module provides:

- A C++ template class for easily integrating ROS2 actions and services into Behavior Trees,
- Navigation-specific behavior tree nodes, and

- a generic BehaviorTreeEngine class that simplifies the integration of BT processing into ROS2 nodes for navigation or higher-level autonomy applications.

> -一个通用的 BehaviorTreeEngine 类，它**简化了 BT 处理到 ROS2 节点的集成**，用于导航或更高级别的自治应用程序。

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-bt-xml.html) for additional parameter descriptions and a list of XML nodes made available in this package. Also review the [Nav2 Behavior Tree Explanation](https://navigation.ros.org/behavior_trees/index.html) pages explaining more context on the default behavior trees and examples provided in this package. A [tutorial](https://navigation.ros.org/plugin_tutorials/docs/writing_new_bt_plugin.html) is also provided to explain how to create a simple BT plugin.

> 请参阅其[配置指南页面]以获取此包中提供的附加参数描述和 XML 节点列表。另请参阅[Nav2 行为树解释]页面解释了有关默认行为树的更多上下文以及此包中提供的示例。A[教程]还提供了解释如何创建一个简单的 BT 插件。

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available planner plugins.

> 请参阅[导航插件列表](https://navigation.ros.org/plugins/index.html)获取当前已知和可用的计划器插件的列表。

## The bt_action_node Template and the Behavior Tree Engine

The [bt_action_node template](include/nav2_behavior_tree/bt_action_node.hpp) allows one to easily integrate a ROS2 action into a BehaviorTree. To do so, one derives from the BtActionNode template, providing the action message type. For example,

> [bt_action_node 模板]允许轻松地将 ROS2 操作集成到 BehaviorTree 中。为此，可以从 BtActionNode 模板派生，提供操作消息类型。例如

```C++
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

class FollowPathAction : public BtActionNode<nav2_msgs::action::FollowPath>
{
    ...
};
```

The resulting node must be registered with the factory in the Behavior Tree engine in order to be available for use in Behavior Trees executed by this engine.

> 生成的节点必须在行为树引擎中的工厂中注册，才能在此引擎执行的行为树中使用。

```C++
BehaviorTreeEngine::BehaviorTreeEngine()
{
    ...

  factory_.registerNodeType<nav2_behavior_tree::FollowPathAction>("FollowPath");

    ...
}
```

Once a new node is registered with the factory, it is now available to the BehaviorTreeEngine and can be used in Behavior Trees. For example, the following simple XML description of a BT shows the FollowPath node in use:

> **一旦向工厂注册了新节点，BehaviorTreeEngine 就可以使用它**，并且可以在行为树中使用它。例如，以下 BT 的简单 XML 描述显示了正在使用的 FollowPath 节点：

```XML
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <ComputePathToPose goal="${goal}"/>
      <FollowPath path="${path}" controller_property="FollowPath"/>
    </Sequence>
  </BehaviorTree>
</root>
```

The BehaviorTree engine has a run method that accepts an XML description of a BT for execution:

> BehaviorTree 引擎有一个 **run 方法，该方法接受 BT 的 XML 描述以供执行**：

```C++
  BtStatus run(
    BT::Blackboard::Ptr & blackboard,
    const std::string & behavior_tree_xml,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));
```

See the code in the [BT Navigator](../nav2_bt_navigator/src/bt_navigator.cpp) for an example usage of the BehaviorTreeEngine.

> 有关 BehaviorTreeEngine 的示例用法，请参阅[BT Navigator]中的代码。

For more information about the behavior tree nodes that are available in the default BehaviorTreeCPP library, see documentation here: https://www.behaviortree.dev/bt_basics/

> 有关默认 BehaviorTreeCPP 库中可用的行为树节点的更多信息，请参阅此处的文档：https://www.behaviortree.dev/bt_basics/
