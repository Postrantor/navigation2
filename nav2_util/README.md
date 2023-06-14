---
tip: translate by openai@2023-06-15 15:50:10
...

# Nav2 Util

The `nav2_util` package contains utilities abstracted from individual packages which may find use in other uses. Some examples of things you'll find here:

> nav2_util 包含了从个别包抽象出来的实用工具，可以用于其他用途。一些你可以在这里找到的例子：

- Geometry utilities for computing distances and values in paths

> - 用于计算路径距离和值的几何实用程序

- A Nav2 specific lifecycle node wrapper for boilerplate code and useful common utilities like `declare_parameter_if_not_declared()`

> 一个 Nav2 特定的生命周期节点包装器，用于提供样板代码和有用的常用实用程序，如`declare_parameter_if_not_declared（）`。

- Simplified service clients
- Simplified action servers
- Transformation and robot pose helpers

The long-term aim is for these utilities to find more permanent homes in other packages (within and outside of Nav2) or migrate to the raw tools made available in ROS 2.

> 长期目标是让这些实用程序找到更多永久的家园（在 Nav2 内部和外部）或迁移到 ROS 2 提供的原始工具中。
