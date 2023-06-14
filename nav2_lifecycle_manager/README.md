---
tip: translate by openai@2023-06-01 17:25:42
...

### Background on lifecycle enabled nodes

Using ROS2’s managed/lifecycle nodes feature allows the system startup to ensure that all required nodes have been instantiated correctly before they begin their execution. Using lifecycle nodes also allows nodes to be restarted or replaced on-line. More details about managed nodes can be found on [ROS2 Design website](https://design.ros2.org/articles/node_lifecycle.html). Several nodes in Nav2, such as map_server, planner_server, and controller_server, are lifecycle enabled. These nodes provide the required overrides of the lifecycle functions: `on_configure()`, `on_activate()`, `on_deactivate()`, `on_cleanup()`, `on_shutdown()`, and `on_error()`.

> 使用 ROS2 的受管理/生命周期节点功能**可以确保系统启动时所有必需的节点都已正确实例化**，然后才开始执行。使用生命周期节点**还允许节点在线重新启动或替换**。有关受管理节点的更多详细信息，请参阅[ROS2 Design website]。Nav2 中的几个节点，如 map_server、planner_server 和 controller_server，都具有生命周期功能。这些节点提供了所需的生命周期函数覆盖：`on_configure()`、`on_activate()`、`on_deactivate()`、`on_cleanup()`、`on_shutdown()`和`on_error()`。

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-lifecycle.html) for additional parameter descriptions.

> 查看[配置指南页面](../../navigation.ros.org/configuration/packages/configuring-lifecycle.md)了解更多参数描述。

> [!NOTE]
> .[这个是怎么用 lifecycle 的示例](..\nav2_planner\src\planner_server.cpp)
>
> ```yaml
> lifecycle_manager:
>   ros__parameters:
>     autostart: true
>     node_names:
>       [
>         "controller_server",
>         "planner_server",
>         "behavior_server",
>         "bt_navigator",
>         "waypoint_follower",
>       ]
>     bond_timeout: 4.0
>     attempt_respawn_reconnection: true
>     bond_respawn_max_duration: 10.0
> ```

### nav2_lifecycle_manager

> [!NOTE]
> 这里还是存在 manager 的概念，区别的应该是与 master 不同。
> manager 可以调用对应的生命周期服务，这个应该是根据系统的策略来安排的，或者是根据具体的业务逻辑实现。考虑实现为类似 QoS 的概念，给出一系列的配置字段，这个是用于定义状态机的；同时类似的给出一些默认的整体配置，是专门针对一些场景的，可以直接调用。

Nav2's lifecycle manager is used to change the states of the lifecycle nodes in order to achieve a controlled _startup_, _shutdown_, _reset_, _pause_, or _resume_ of the navigation stack. The lifecycle manager presents a `lifecycle_manager/manage_nodes` service, from which clients can invoke the startup, shutdown, reset, pause, or resume functions. Based on this service request, the lifecycle manager calls the necessary lifecycle services in the lifecycle managed nodes. Currently, the RVIZ panel uses this `lifecycle_manager/manage_nodes` service when user presses the buttons on the RVIZ panel (e.g.,startup, reset, shutdown, etc.), but it is meant to be called on bringup through a production system application.

> **Nav2 的生命周期管理器用于更改生命周期节点的状态，以实现导航堆栈的受控启动、关闭、重置、暂停或恢复**。生命周期管理器提供了一个`lifecycle_manager/manage_nodes`服务，客户端可以调用启动、关闭、重置、暂停或恢复功能。**根据此服务请求，生命周期管理器(lifecycle manager)会调用生命周期管理节点(lifecycle managed nodes)中必要的生命周期服务**。
> 目前，RVIZ 面板在用户按下 RVIZ 面板上的按钮时(例如启动、重置、关闭等)会调用此`lifecycle_manager/manage_nodes`服务，但它旨在**通过生产系统应用程序在启动时调用**。

In order to start the navigation stack and be able to navigate, the necessary nodes must be configured and activated. Thus, for example when _startup_ is requested from the lifecycle manager's `manage_nodes` service, the lifecycle managers calls _configure()_ and _activate()_ on the lifecycle enabled nodes in the node list. These are all transitioned in ordered groups for bringup transitions, and reverse ordered groups for shutdown transitions.

> **为了启动导航堆栈并能够导航，必须配置和激活必要的节点**。因此，例如，当从生命周期管理器的 `manage_nodes` 服务请求 _startup_ 时，生命周期管理器调用 _configure()_ 和 _activate()_ 在节点列表中的生命周期启用节点。这些都转换为有序组以进行启动转换，和相反顺序组以进行关机转换。

> [!NOTE]
> 这里给出 _startup_ 的概念，下属包括 _configure()_ 和 _activate()_。这样**对已有的一些状态进行一些组合**实现具体的业务的要求？

The lifecycle manager has a default nodes list for all the nodes that it manages. This list can be changed using the lifecycle manager’s _“node_names”_ parameter.

> 生命周期管理器为所有节点提供默认节点列表。可以使用生命周期管理器的“node_names”参数来更改此列表。

The diagram below shows an _example_ of a list of managed nodes, and how it interfaces with the lifecycle manager.

> 下图展示了一个管理节点列表的 _示例_，以及它如何与生命周期管理器进行交互。
> <img src="./doc/diagram_lifecycle_manager.JPG" title="" width="100%" align="middle">

The UML diagram below shows the sequence of service calls once the _startup_ is requested from the lifecycle manager.

> 下面的 UML 图显示了从生命周期管理器请求 _startup_ 后的服务调用序列。

<img src="./doc/uml_lifecycle_manager.JPG" title="Lifecycle manager UML diagram" width="100%" align="middle">
