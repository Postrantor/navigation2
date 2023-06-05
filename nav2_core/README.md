# Nav2 Core

This package hosts the abstract interface (virtual base classes) for plugins to be used with the following:

> 该软件包托管了要使用以下插件的抽象接口（虚拟基类）：

- navigators (e.g., `navigate_to_pose`)
- global planner (e.g., `nav2_navfn_planner`)
- controller (e.g., path execution controller, e.g `nav2_dwb_controller`)
- smoother (e.g., `nav2_ceres_costaware_smoother`)
- goal checker (e.g. `simple_goal_checker`)
- behaviors (e.g. `drive_on_heading`)
- progress checker (e.g. `simple_progress_checker`)
- waypoint task executor (e.g. `take_pictures`)
- exceptions in planning and control

The purposes of these plugin interfaces are to create a separation of concern from the system software engineers and the researcher / algorithm designers. Each plugin type is hosted in a "task server" (e.g. planner, recovery, control servers) which handles requests and multiple algorithm plugin instances. The plugins are used to compute a value back to the server without having to worry about ROS 2 actions, topics, or other software utilities. A plugin designer can simply use the tools provided in the API to do their work, or create new ones if they like internally to gain additional information or capabilities.

> 这些插件界面的目的是与系统软件工程师和研究人员 /算法设计师建立关注点。每种插件类型都托管在“任务服务器”（例如计划者，恢复，控制服务器）中，该插件处理请求和多个算法插件实例。该插件用于将值重新计算回服务器，而不必担心ROS 2操作，主题或其他软件实用程序。插件设计人员可以简单地使用API中提供的工具来完成其工作，或者如果他们愿意在内部愿意获得其他信息或功能，则可以创建新的工作。
