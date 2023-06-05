---
tip: translate by baidu@2023-06-06 08:54:42
...

# Instructions on using Groot

[Groot](https://github.com/BehaviorTree/Groot) is the companion application of [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) to create, edit, and monitor behavior trees.

> [Groot]是[BehaviorTree.CPP]的配套应用程序创建、编辑和监视行为树。

Note: Currently fully supports visualization of the behavior trees. It also supports the creation of custom nodes except control flow nodes. Support for custom control flow nodes and real-time monitoring is under development.

> 注意：目前完全**支持行为树的可视化。它还支持除控制流节点以外的自定义节点的创建**。正在开发**对自定义控制流节点和实时监控**的支持。

### BehaviorTree visualization

To visualize the behavior trees using Groot:

> 要使用 Groot 可视化行为树，请执行以下操作：

1. Open Groot in editor mode
2. Select the `Load palette from file` option (import button) near the top left corner.
3. Open the file `/path/to/nav2/nav2_behavior_tree/nav2_tree_nodes.xml` to import all the behavior tree nodes used for navigation.
4. Select `Load tree` option near the top left corner
5. Browse the tree you want to visualize the select ok.

> 1. 在编辑器模式下打开 Groot
> 2. 选择左上角附近的“从文件加载调色板”选项（导入按钮）。
> 3. 打开文件`/path/to/nav2/nav2_behavior_tree/nav2_tree_nodes.xml`导入所有用于导航的行为树节点。
> 4. 选择左上角附近的“加载树”选项
> 5. 浏览要可视化的树，然后选择“确定”。
