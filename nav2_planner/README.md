---
tip: translate by openai@2023-06-14 17:21:32
...

# Nav2 Planner

The Nav2 planner is a Task Server in Nav2 that implements the `nav2_behavior_tree::ComputePathToPose` interface.

> Nav2 规划器是 Nav2 中的一个任务服务器，它实现了 `nav2_behavior_tree::ComputePathToPose` 接口。

A planning module implementing the `nav2_behavior_tree::ComputePathToPose` interface is responsible for generating a feasible path given start and end robot poses. It loads a map of potential planner plugins to do the path generation in different user-defined situations.

> 一个实现`nav2_behavior_tree::ComputePathToPose`接口的规划模块负责根据起始机器人姿态生成可行路径。它加载了一个潜在规划插件的地图，以便在不同的用户定义的情况下进行路径生成。

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available planner plugins.

> 请参阅[导航插件列表]，了解当前已知和可用的规划器插件列表。

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-planner-server.html) for additional parameter descriptions and a [tutorial about writing planner plugins](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html).

> 请参阅[配置指南页面]以获取附加参数描述，以及[关于编写规划器插件的教程]。
