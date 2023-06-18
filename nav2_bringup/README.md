---
tip: translate by openai@2023-06-19 11:51:56
...

# nav2_bringup

The `nav2_bringup` package is an example bringup system for Nav2 applications.

> 这个`nav2_bringup`包是一个 Nav2 应用程序的示例启动系统。

This is a very flexible example for nav2 bringup that can be modified for different maps/robots/hardware/worlds/etc. It is our expectation for an application specific robot system that you're mirroring `nav2_bringup` package and modifying it for your specific maps/robots/bringup needs. This is an applied and working demonstration for the default system bringup with many options that can be easily modified.

> 这是一个非常灵活的 nav2 启动示例，可以针对不同的地图/机器人/硬件/世界等进行修改。我们期望您可以镜像`nav2_bringup`软件包，并针对您的特定地图/机器人/启动需求进行修改。这是默认系统启动的一个应用和工作演示，具有许多可以轻松修改的选项。

Usual robot stacks will have a `<robot_name>_nav` package with config/bringup files and this is that for the general case to base a specific robot system off of.

> 一般的机器人堆栈都会有一个`<robot_name>_nav`包，包含配置/启动文件，这是一般情况下基于特定机器人系统的基础。

Dynamically composed bringup (based on [ROS2 Composition](https://docs.ros.org/en/galactic/Tutorials/Composition.html)) is optional for users. It can be used to compose all Nav2 nodes in a single process instead of launching these nodes separately, which is useful for embedded systems users that need to make optimizations due to harsh resource constraints. Dynamically composed bringup is used by default, but can be disabled by using the launch argument `use_composition:=False`.

> 动态组合启动（基于[ROS2 组合](https://docs.ros.org/en/galactic/Tutorials/Composition.html))对用户是可选的。它可**用于在单个进程中组合所有 Nav2 节点，而不是将这些节点单独启动，这对于因严格的资源限制而需要进行优化的嵌入式系统用户非常有用**。默认情况下使用动态组合启动，但可以使用启动参数`use_composition:=False`来禁用它。

- Some discussions about performance improvement of composed bringup could be found here: https://discourse.ros.org/t/nav2-composition/22175.

> 在这里可以找到关于组合启动性能改进的一些讨论：https://discourse.ros.org/t/nav2-composition/22175。

To use, please see the Nav2 [Getting Started Page](https://navigation.ros.org/getting_started/index.html) on our documentation website. Additional [tutorials will help you](https://navigation.ros.org/tutorials/index.html) go from an initial setup in simulation to testing on a hardware robot, using SLAM, and more.

> 要使用，请查看我们文档网站上的 Nav2 [入门页面](https://navigation.ros.org/getting_started/index.html)。附加的[教程将帮助您](https://navigation.ros.org/tutorials/index.html)从模拟中的初始设置开始，到使用 SLAM 在硬件机器人上进行测试等等。

Note:

- gazebo should be started with both libgazebo_ros_init.so and libgazebo_ros_factory.so to work correctly.

> 应该同时启动 libgazebo_ros_init.so 和 libgazebo_ros_factory.so 才能正常工作。

- spawn_entity node could not remap /tf and /tf_static to tf and tf_static in the launch file yet, used only for multi-robot situations. Instead it should be done as remapping argument `<remapping>/tf:=tf</remapping> <remapping>/tf_static:=tf_static</remapping>` under ros2 tag in each plugin which publishs transforms in the SDF file. It is essential to differentiate the tf's of the different robot.

> spawn_entity 节点尚未能够在 launch 文件中将/tf 和/tf_static 重新映射为 tf 和 tf_static，仅用于多机器人情况。相反，它应该作为重新映射参数`<remapping>/tf:=tf</remapping> <remapping>/tf_static:=tf_static</remapping>`在 SDF 文件中发布变换的每个插件的 ros2 标签下进行。区分不同机器人的 tf 至关重要。
