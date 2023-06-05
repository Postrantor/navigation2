## 请问这个类中的行为树是如何构建的？

在这个类中，行为树是从 XML 文件构建的。具体的构建过程发生在 `on_configure` 生命周期回调函数中。虽然代码段中没有提供 `on_configure` 函数的具体实现，但根据注释和通常的 ROS2 行为树使用方法，我可以为您提供一个关于如何构建行为树的概述。

在 `on_configure` 函数中，您可以执行以下操作来构建行为树：

1. 首先，从参数服务器获取包含行为树定义的 XML 文件的路径。这可以通过获取 ROS2 参数来完成，例如：

   ```cpp
   std::string bt_xml_filename;
   get_parameter("bt_xml_filename", bt_xml_filename);
   ```

2. 使用 Behavior Tree 库提供的工具，从 XML 文件中读取行为树。这可以通过创建一个 `BT::Tree` 对象来完成。例如：

   ```cpp
   BT::Tree tree = BT::buildTreeFromFile(bt_xml_filename);
   ```

   这将从给定的 XML 文件构建行为树，并将其存储在 `tree` 变量中。

3. 为行为树注册自定义节点，例如行为、条件和修饰符。这可以通过创建一个 `BT::NodeFactory` 对象并使用 `registerNodeType` 方法来完成。例如：

   ```cpp
   BT::NodeFactory factory;
   factory.registerNodeType<MyCustomAction>("MyCustomAction");
   ```

   这将注册一个名为 "MyCustomAction" 的自定义行为。

4. 在行为树的构建过程中，您还可以向构建的树传递所需的黑板（也称为“输入端口”）。黑板是一种在树中节点之间共享数据的方式。例如，可以将机器人的当前位置和目标位置作为黑板变量传递给行为树。要将输入端口添加到树中，请使用 `BT::assignPorts` 函数。例如：

   ```cpp
   BT::assignPorts(tree.rootNode(), {BT::InputPort<MyDataType>("my_input_port", my_data)});
   ```

   这将在树的根节点上创建一个名为 "my_input_port" 的输入端口，并将 `my_data` 对象作为其值。

完成上述步骤后，行为树将成功构建，并准备好在后续的导航任务中执行。在实际的 `BtNavigator` 类实现中，您可能还需要存储构建的行为树以供后续使用，例如在 `on_activate` 生命周期回调函数中启动行为树执行。

## 您能否提供一个示例 XML 文件，以便我更好地理解如何构建行为树？

当然可以！下面是一个简单的行为树 XML 示例，它包含了一个顺序节点（Sequence）和两个自定义行为节点（CustomAction1 和 CustomAction2）。顺序节点按顺序执行其子节点，只有当一个子节点成功时，它才会执行下一个子节点。如果所有子节点都成功，顺序节点则成功；如果有子节点失败，顺序节点则失败。

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="sequence">
      <CustomAction1 name="action1"/>
      <CustomAction2 name="action2"/>
    </Sequence>
  </BehaviorTree>
</root>
```

这个示例只是一个简单的行为树，但它为您展示了如何在 XML 文件中定义树结构。您可以将此 XML 文件作为基础，根据需要添加更多的节点（例如选择节点、并行节点等）和自定义行为或条件。

请注意，`CustomAction1` 和 `CustomAction2` 是示例中定义的自定义行为节点。在实际代码中，您需要为这些节点编写相应的 C++ 类并在 `BtNavigator` 类的 `on_configure` 函数中使用 `registerNodeType` 方法将它们注册到行为树库。这将允许行为树在运行时找到并执行这些自定义节点。

## 如何在 C++ 中编写自定义行为节点并将其注册到行为树库中？

要在 C++ 中编写自定义行为节点并将其注册到行为树库中，请按照以下步骤操作：

1. 首先，包含 Behavior Tree 库的相关头文件，并定义您的自定义行为类。该类应继承自 `BT::SyncActionNode`、`BT::AsyncActionNode` 或其他适当的基类。例如，对于一个同步行为节点，您可以这样定义：

   ```cpp
   #include "behaviortree_cpp_v3/action_node.h"

   class CustomAction1 : public BT::SyncActionNode {
   public:
     // 构造函数，用于初始化节点
     CustomAction1(const std::string& name, const BT::NodeConfiguration& config)
         : BT::SyncActionNode(name, config) {}

     // 必须覆盖的虚函数，实现节点的具体行为
     BT::NodeStatus tick() override;
   };
   ```

2. 实现 `tick()` 函数，该函数包含您的自定义行为的核心逻辑。在这个函数中，您可以执行任何需要的操作，例如移动机器人、获取传感器数据等。当操作完成时，返回一个 `BT::NodeStatus`，表示操作的成功、失败或正在运行状态。例如：

   ```cpp
   BT::NodeStatus CustomAction1::tick() {
     // 在这里实现您的自定义逻辑
     // ...

     // 如果操作成功完成，返回 BT::NodeStatus::SUCCESS
     // 如果操作失败，返回 BT::NodeStatus::FAILURE
     // 如果操作仍在进行中（仅适用于异步节点），返回 BT::NodeStatus::RUNNING
     return BT::NodeStatus::SUCCESS;
   }
   ```

3. 最后，在 `BtNavigator` 类的 `on_configure` 函数中，使用 `registerNodeType` 方法将您的自定义节点注册到行为树库中。这将允许行为树在运行时找到并执行这些自定义节点。例如：

   ```cpp
   BT::BehaviorTreeFactory factory;

   // 将 CustomAction1 注册到行为树库中
   factory.registerNodeType<CustomAction1>("CustomAction1");

   // 使用 factory 创建行为树
   auto tree = factory.createTreeFromText(bt_xml_string);
   ```

   注意，这里的 `"CustomAction1"` 字符串应与 XML 文件中的节点名称相匹配。

按照上述步骤，您可以创建任意多个自定义行为节点，并在行为树的 XML 文件中使用它们。记住为每个自定义节点编写相应的 C++ 类，并在 `BtNavigator` 类的 `on_configure` 函数中注册它们。这将确保行为树在运行时可以正确找到并执行您的自定义节点。

## 您能否解释一下黑板是如何在行为树中使用的？

当然可以！黑板（Blackboard）是行为树中的一种数据共享机制，允许在树的不同节点之间传递数据。黑板可以看作是一个键-值存储，其中键是字符串，值可以是任意类型的数据。在 Behavior Tree 库中，黑板被称为“输入/输出端口”。

以下是如何在行为树中使用黑板的概述：

1. **定义输入/输出端口**：在自定义节点的 C++ 类中，您可以使用 `BT::InputPort` 和 `BT::OutputPort` 宏定义输入和输出端口。例如，为了在自定义行为中定义一个名为 `target_position` 的输入端口，您可以这样写：

   ```cpp
   class CustomAction1 : public BT::SyncActionNode {
   public:
     // 构造函数，用于初始化节点
     CustomAction1(const std::string& name, const BT::NodeConfiguration& config)
         : BT::SyncActionNode(name, config) {
       getInput("target_position", target_position_);
     }

   private:
     // 定义输入端口
     BT::InputPort<geometry_msgs::msg::Point>("target_position");

     geometry_msgs::msg::Point target_position_;
   };
   ```

   这将在您的自定义节点中创建一个名为 `target_position` 的输入端口。

2. **在 XML 文件中设置端口值**：在行为树的 XML 文件中，您可以使用 `<Param>` 标签为输入/输出端口设置值。例如，要将名为 `goal_position` 的黑板变量传递给上面定义的 `target_position` 输入端口，您可以这样写：

   ```xml
   <CustomAction1 name="action1">
     <Param name="target_position" value="{goal_position}"/>
   </CustomAction1>
   ```

   在这个例子中，`{goal_position}` 是一个特殊的语法，表示从黑板中获取名为 `goal_position` 的变量的值。这将把 `goal_position` 的值传递给自定义行为的 `target_position` 输入端口。

3. **使用黑板数据**：在自定义节点的 `tick()` 函数中，您可以使用输入/输出端口的数据执行操作。您可以直接访问在类中定义的端口变量。例如：

   ```cpp
   BT::NodeStatus CustomAction1::tick() {
     // 使用 target_position_ 输入端口的数据执行操作
     // ...

     return BT::NodeStatus::SUCCESS;
   }
   ```

4. **设置黑板数据**：对于输出端口，您可以在 `tick()` 函数中设置值，然后将其传递回黑板。例如：

   ```cpp
   class CustomAction2 : public BT::SyncActionNode {
   public:
     // 构造函数，用于初始化节点
     CustomAction2(const std::string& name, const BT::NodeConfiguration& config)
         : BT::SyncActionNode(name, config) {}

     // 定义输出端口
     BT::OutputPort<geometry_msgs::msg::Point>("new_position");

   private:
     BT::NodeStatus tick() override;
   };

   BT::NodeStatus CustomAction2::tick() {
     geometry_msgs::msg::Point new_position;
     // 计算新位置
     // ...

     // 设置输出端口的值
     setOutput("new_position", new_position);

     return BT::NodeStatus::SUCCESS;
   }
   ```

   这将创建一个名为 `new_position` 的输出端口，并在 `tick()` 函数中设置其值。然后，您可以在 XML 文件中将其传递给其他节点的输入端口。

通过使用黑板，您可以在行为树中的不同节点之间共享数据，使得节点可以基于共享数据执行决策和操作。这有助于提高行为树的灵活性和可重用性。

## 你能举例说明如何在行为树中使用黑板吗？

当然可以！下面是一个使用黑板（Blackboard）的行为树示例。在这个示例中，我们有一个行为树，它包含一个顺序节点（Sequence）和三个自定义行为节点：`SetTargetPosition`、`MoveToTarget` 和 `ReportPosition`。我们将使用黑板在这些节点之间共享数据。

首先，我们创建如下行为树的 XML 文件：

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="sequence">
      <SetTargetPosition name="set_target"/>
      <MoveToTarget name="move_to_target">
        <Param name="target_position" value="{goal_position}"/>
      </MoveToTarget>
      <ReportPosition name="report_position">
        <Param name="current_position" value="{goal_position}"/>
      </ReportPosition>
    </Sequence>
  </BehaviorTree>
</root>
```

在这个 XML 文件中，我们使用 `<Param>` 标签设置输入端口的值。`MoveToTarget` 和 `ReportPosition` 节点都使用名为 `goal_position` 的黑板变量。

接下来，我们定义三个自定义行为节点的 C++ 类：

1. **SetTargetPosition**：此节点负责设置目标位置并将其保存在黑板中。

   ```cpp
   #include "behaviortree_cpp_v3/action_node.h"

   class SetTargetPosition : public BT::SyncActionNode {
   public:
     SetTargetPosition(const std::string& name, const BT::NodeConfiguration& config)
         : BT::SyncActionNode(name, config) {}

     // 定义输出端口
     BT::OutputPort<geometry_msgs::msg::Point>("goal_position");

     BT::NodeStatus tick() override {
       geometry_msgs::msg::Point target_position;
       // 设置目标位置
       // ...

       // 将目标位置保存到黑板中
       setOutput("goal_position", target_position);
       return BT::NodeStatus::SUCCESS;
     }
   };
   ```

2. **MoveToTarget**：此节点读取目标位置并执行移动操作。

   ```cpp
   #include "behaviortree_cpp_v3/action_node.h"

   class MoveToTarget : public BT::SyncActionNode {
   public:
     MoveToTarget(const std::string& name, const BT::NodeConfiguration& config)
         : BT::SyncActionNode(name, config) {
       getInput("target_position", target_position_);
     }

     // 定义输入端口
     BT::InputPort<geometry_msgs::msg::Point>("target_position");

   private:
     geometry_msgs::msg::Point target_position_;

     BT::NodeStatus tick() override {
       // 使用 target_position_ 执行移动操作
       // ...

       return BT::NodeStatus::SUCCESS;
     }
   };
   ```

3. **ReportPosition**：此节点读取当前位置并报告。

   ```cpp
   #include "behaviortree_cpp_v3/action_node.h"

   class ReportPosition : public BT::SyncActionNode {
   public:
     ReportPosition(const std::string& name, const BT::NodeConfiguration& config)
         : BT::SyncActionNode(name, config) {
       getInput("current_position", current_position_);
     }

     // 定义输入端口
     BT::InputPort<geometry_msgs::msg::Point>("current_position");

   private:
     geometry_msgs::msg::Point current_position_;

     BT::NodeStatus tick() override {
       // 使用 current_position_ 报告位置
       // ...

       return BT::NodeStatus::SUCCESS;
     }
   };
   ```

最后，在 `BtNavigator` 类的 `on_configure` 函数中，使用 `registerNodeType` 方法将自定义节点注册到行为树库中。

```cpp
BT::BehaviorTreeFactory factory;

// 将自定义节点注册到行为树库中
factory.registerNodeType<SetTargetPosition>("SetTargetPosition");
factory.registerNodeType<MoveToTarget>("MoveToTarget");
factory.registerNodeType<ReportPosition>("ReportPosition");

// 使用 factory 创建行为树
auto tree = factory.createTreeFromText(bt_xml_string);
```

这个示例展示了如何在行为树中使用黑板来在不同节点之间共享数据。`SetTargetPosition` 节点将目标位置设置到黑板中，然后 `MoveToTarget` 和 `ReportPosition` 节点分别读取并使用该值。这样，您可以根据共享的数据执行特定的操作和决策，从而提高行为树的灵活性和可重用性。

## 使用黑板变量如何提高行为树的灵活性和可重用性？

使用黑板变量可以在行为树各个节点之间共享数据，提高行为树的灵活性和可重用性。具体来说，以下是黑板变量如何实现这一点的一些方面：

1. **参数化节点**：通过使用黑板变量作为节点的输入参数，您可以使节点更加通用和可配置。例如，一个名为 `MoveToTarget` 的行为节点可以从黑板接收目标位置，而不是在节点中硬编码。这样，您可以在不同情况下重复使用相同的节点，只需更改传递给它的目标位置即可。

2. **灵活的数据流**：黑板允许在树中的不同节点之间灵活地传递数据，从而使得您可以轻松地在各个节点之间共享信息。这为行为树提供了更强大的表达能力，使得您可以根据共享的数据执行特定的操作和决策。

3. **解耦行为和数据**：通过将数据与节点行为分离，黑板允许您更好地封装节点的逻辑，使其聚焦于任务本身，而不是如何获取和管理数据。这使得您的行为树更容易设计、理解和维护。

4. **易于调试和监控**：黑板变量提供了一种简单的方式来监控和调试行为树中的数据流。您可以轻松地查看黑板中的各个变量的值，了解树的当前状态和执行过程。

5. **支持条件节点**：黑板变量使得您可以轻松地实现基于数据的条件节点。例如，您可以创建一个条件节点来检查黑板上的某个变量是否满足特定条件。这为行为树提供了更强大的决策能力，使得您可以根据共享的数据执行特定的分支和操作。

总之，黑板变量提高了行为树的灵活性和可重用性，使得您可以创建更通用、可配置和可维护的行为树。这对于实现复杂的任务和决策过程至关重要。

## 使用黑板的示例

当然可以！让我们以一个简单的机器人导航任务为例来说明如何使用黑板变量。在这个例子中，我们有一个行为树，它包含一个顺序节点和三个自定义行为节点：`GetTargetPosition`、`MoveToTarget` 和 `CheckBattery`。我们将使用黑板在这些节点之间共享数据。

首先，我们创建如下行为树的 XML 文件：

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="sequence">
      <GetTargetPosition name="get_target"/>
      <MoveToTarget name="move_to_target">
        <Param name="target_position" value="{goal_position}"/>
      </MoveToTarget>
      <CheckBattery name="check_battery"/>
    </Sequence>
  </BehaviorTree>
</root>
```

在这个 XML 文件中，我们使用 `<Param>` 标签设置输入端口的值。`MoveToTarget` 节点使用名为 `goal_position` 的黑板变量。

接下来，我们定义三个自定义行为节点的 C++ 类：

1. **GetTargetPosition**：此节点负责获取目标位置并将其保存在黑板中。

   ```cpp
   #include "behaviortree_cpp_v3/action_node.h"

   class GetTargetPosition : public BT::SyncActionNode {
   public:
     GetTargetPosition(const std::string& name, const BT::NodeConfiguration& config)
         : BT::SyncActionNode(name, config) {}

     // 定义输出端口
     BT::OutputPort<geometry_msgs::msg::Point>("goal_position");

     BT::NodeStatus tick() override {
       geometry_msgs::msg::Point target_position;
       // 获取目标位置，例如从地图服务器处获取
       // ...

       // 将目标位置保存到黑板中
       setOutput("goal_position", target_position);
       return BT::NodeStatus::SUCCESS;
     }
   };
   ```

2. **MoveToTarget**：此节点读取目标位置并移动机器人。

   ```cpp
   #include "behaviortree_cpp_v3/action_node.h"

   class MoveToTarget : public BT::SyncActionNode {
   public:
     MoveToTarget(const std::string& name, const BT::NodeConfiguration& config)
         : BT::SyncActionNode(name, config) {
       getInput("target_position", target_position_);
     }

     // 定义输入端口
     BT::InputPort<geometry_msgs::msg::Point>("target_position");

   private:
     geometry_msgs::msg::Point target_position_;

     BT::NodeStatus tick() override {
       // 使用 target_position_ 控制机器人移动到目标位置
       // ...

       return BT::NodeStatus::SUCCESS;
     }
   };
   ```

3. **CheckBattery**：此节点检查机器人的电池状态。

   ```cpp
   #include "behaviortree_cpp_v3/condition_node.h"

   class CheckBattery : public BT::ConditionNode {
   public:
     CheckBattery(const std::string& name, const BT::NodeConfiguration& config)
         : BT::ConditionNode(name, config) {}

     BT::NodeStatus tick() override {
       // 检查机器人的电池状态
       // ...

       if (battery_status_ok) {
         return BT::NodeStatus::SUCCESS;
       } else {
         return BT::NodeStatus::FAILURE;
       }
     }
   };
   ```

最后，在 `BtNavigator` 类的 `on_configure` 函数中，使用 `registerNodeType` 方法将自定义节点注册到行为树库中。

```cpp
BT::BehaviorTreeFactory factory;

// 将自定义节点注册到行为树库中
factory.registerNodeType<GetTargetPosition>("GetTargetPosition");
factory.registerNodeType<MoveToTarget>("MoveToTarget");
factory.registerNodeType<CheckBattery>("CheckBattery");

// 使用 factory 创建行为树
auto tree = factory.createTreeFromText(bt_xml_string);
```

这个示例展示了如何在一个简单的机器人导航任务中使用黑板变量。`GetTargetPosition` 节点从地图服务器获取目标位置并将其存储到黑板中。然后，`MoveToTarget` 节点读取黑板上的目标位置并控制机器人移动。最后，`CheckBattery` 节点检查机器人的电池状态。通过使用黑板变量，我们使得行为树更加灵活，并能轻松地在节点之间共享数据。这对于实现复杂的任务和决策过程是至关重要的。

## 如何在行为树中使用条件节点？

在行为树中，条件节点（Condition Nodes）用于根据某些条件决定执行哪个子树。条件节点通常位于控制流节点（例如序列节点（Sequence）、选择节点（Selector）或并行节点（Parallel））的子节点，这些控制流节点基于条件节点的返回状态（成功、失败或运行中）来决定如何执行它们的其他子节点。

条件节点可以是自定义的，也可以是预定义的。预定义的条件节点通常用于检查黑板上的变量是否满足某些条件。自定义条件节点则可以用于更复杂的条件检查，例如检查机器人的状态或环境信息。

以下是一个使用条件节点的示例行为树：

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Selector name="main_selector">
      <Sequence name="pick_up_object_sequence">
        <CheckObjectAvailable name="check_object_available"/>
        <PickUpObject name="pick_up_object"/>
      </Sequence>
      <Sequence name="navigate_sequence">
        <CheckTargetAvailable name="check_target_available"/>
        <NavigateToTarget name="navigate_to_target"/>
      </Sequence>
    </Selector>
  </BehaviorTree>
</root>
```

在这个示例中，我们有一个选择节点（Selector），它有两个序列节点（Sequence）作为子节点。每个序列节点的第一个子节点都是一个条件节点：`CheckObjectAvailable` 和 `CheckTargetAvailable`。这些条件节点分别检查是否有可用的对象需要拾取，以及是否有可用的导航目标。

选择节点会按顺序评估它的子节点。当 `CheckObjectAvailable` 条件节点成功时，选择节点将执行 `pick_up_object_sequence` 子树。否则，选择节点将尝试执行下一个子节点 `navigate_sequence`。如果 `CheckTargetAvailable` 条件节点成功，选择节点将执行 `navigate_sequence` 子树。

下面是一个简单的自定义条件节点示例：

```cpp
#include "behaviortree_cpp_v3/condition_node.h"

class CheckObjectAvailable : public BT::ConditionNode {
public:
  CheckObjectAvailable(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ConditionNode(name, config) {}

  BT::NodeStatus tick() override {
    // 检查对象是否可用，例如查询对象数据库
    bool object_available = ...;

    if (object_available) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }
};
```

在这个例子中，`CheckObjectAvailable` 条件节点检查对象是否可用。如果对象可用，节点返回成功状态；否则，返回失败状态。我们可以类似地实现 `CheckTargetAvailable` 条件节点，以检查导航目标是否可用。

总之，条件节点在行为树中允许您根据特定条件执行不同的子树。通过使用条件节点，您可以轻松地实现基于数据的决策和分支。这对于实现复杂的任务和决策过程至关重要。

## 如何访问自定义节点中输入端口的值？

## 访问 XML 文件中的黑板变量的语法是什么？

在行为树的 XML 文件中，您可以使用花括号 `{}` 来引用黑板变量。通过将变量名放在花括号内，您可以在 `Param` 标签中访问黑板上的值。这种语法允许您在不同行为节点之间传递和共享数据。

以下是一个简单的示例，说明如何在 XML 文件中引用黑板变量：

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="sequence">
      <GetTargetPosition name="get_target"/>
      <MoveToTarget name="move_to_target">
        <Param name="target_position" value="{goal_position}"/>
      </MoveToTarget>
    </Sequence>
  </BehaviorTree>
</root>
```

在这个示例中，`MoveToTarget` 节点有一个名为 `target_position` 的输入端口。我们使用 `<Param>` 标签设置输入端口的值，并通过 `{goal_position}` 引用黑板变量 `goal_position`。这意味着当 `MoveToTarget` 节点执行时，它将使用黑板上名为 `goal_position` 的变量作为 `target_position` 输入端口的值。

需要注意的是，这种语法仅适用于 XML 文件中的 `Param` 标签。在 C++ 代码中，您需要使用 `getInput`、`setInput`、`getOutput` 或 `setOutput` 方法来访问和修改黑板变量。

## 行为树中的选择节点和并行节点是如何工作的？

在行为树中，选择节点（Selector）和并行节点（Parallel）是两种常见的控制流节点，用于组织和控制行为树的执行。以下是它们的工作原理：

**选择节点（Selector）：**

选择节点按顺序评估其子节点，直到找到一个返回成功（`BT::NodeStatus::SUCCESS`）或运行中（`BT::NodeStatus::RUNNING`）的子节点。选择节点在以下情况下返回成功：

- 当一个子节点返回成功时，选择节点立即返回成功，不再评估其余子节点。

选择节点在以下情况下返回失败：

- 当所有子节点都返回失败时，选择节点返回失败。

选择节点在以下情况下返回运行中：

- 当一个子节点返回运行中时，选择节点立即返回运行中，不再评估其余子节点。

选择节点通常用于实现具有多种备选方案的任务，当某个备选方案成功时，任务就会成功。

**并行节点（Parallel）：**

并行节点同时执行其所有子节点，并根据给定的策略（成功阈值和失败阈值）确定其返回状态。当一个子节点完成执行，返回状态为成功或失败时，更新并行节点的成功计数或失败计数。并行节点在以下情况下返回成功：

- 当成功计数达到或超过成功阈值时，立即返回成功。

并行节点在以下情况下返回失败：

- 当失败计数达到或超过失败阈值时，立即返回失败。

如果并行节点没有返回成功或失败，它将继续执行其子节点并返回运行中。

并行节点通常用于实现需要同时执行多个任务的场景，例如同时进行导航和避障。通过调整成功阈值和失败阈值，可以控制并行节点的决策策略。

需要注意的是，行为树的实现可能会因库而异，例如 [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) 和 [ROS-Behavior-Tree](https://github.com/miccol/ROS-Behavior-Tree)，在实际使用时，请参考您正在使用的行为树库的文档。

## 如何在行为树中添加条件节点以便更好地控制行为？

## 行为树中还有哪些常见的控制流节点？

在行为树中，除了选择节点（Selector）和并行节点（Parallel）之外，还有一些其他常见的控制流节点，用于组织和控制行为树的执行。以下是一些常见的控制流节点：

1. **序列节点（Sequence）：**

   序列节点按顺序执行其子节点。当一个子节点返回成功（`BT::NodeStatus::SUCCESS`），序列节点将执行下一个子节点。序列节点在以下情况下返回成功：

   - 当所有子节点都返回成功时，序列节点返回成功。

   序列节点在以下情况下返回失败：

   - 当一个子节点返回失败（`BT::NodeStatus::FAILURE`）时，序列节点立即返回失败，不再执行其余子节点。

   序列节点在以下情况下返回运行中：

   - 当一个子节点返回运行中（`BT::NodeStatus::RUNNING`）时，序列节点立即返回运行中，不再执行其余子节点。

   序列节点通常用于实现需要按顺序执行的任务，例如先拾取物体，然后将物体放置在目标位置。

2. **重试节点（Retry）：**

   重试节点尝试执行其子节点一定次数，直到子节点成功或达到最大重试次数。重试节点在以下情况下返回成功：

   - 当子节点返回成功时，重试节点立即返回成功。

   重试节点在以下情况下返回失败：

   - 当达到最大重试次数，且子节点仍然返回失败时，重试节点返回失败。

   重试节点在以下情况下返回运行中：

   - 当子节点返回运行中时，重试节点立即返回运行中。

   重试节点通常用于实现可能需要多次尝试才能成功的任务，例如尝试连接到远程服务器。

3. **装饰节点（Decorator）：**

   装饰节点是一种特殊类型的控制流节点，它只有一个子节点。装饰节点会在执行子节点之前或之后执行一些额外的操作，例如条件检查、计时或状态转换。根据具体的装饰节点实现，装饰节点可能会修改其子节点的返回状态（成功、失败或运行中）。

   装饰节点通常用于在不修改子节点的情况下为其添加额外的功能或约束。例如，可以使用一个装饰节点实现超时功能，当子节点执行超过给定时间限制时，装饰节点返回失败。

以上是一些常见的控制流节点。实际上，您可以根据需要自定义控制流节点来满足您的特定需求。在实际使用时，请参考您正在使用的行为树库的文档，了解库中提供的控制流节点和相关功能。

## 请举例说明如何使用重试节点。

假设我们有一个行为树，该树需要执行一个名为 `ConnectToServer` 的任务。这个任务尝试连接到远程服务器，但有时可能因为网络问题而失败。我们希望在失败的情况下重试连接，最多尝试 3 次。为了实现这个需求，我们可以使用重试节点（Retry）。

以下是一个简化的 XML 示例，展示了如何在行为树中使用重试节点：

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="sequence">
      <Retry num_attempts="3" name="retry_connect">
        <ConnectToServer name="connect_to_server"/>
      </Retry>
      <PerformTask name="perform_task"/>
    </Sequence>
  </BehaviorTree>
</root>
```

在这个示例中，我们首先创建一个序列节点（Sequence），包含两个子节点。第一个子节点是一个重试节点（Retry），名为 `retry_connect`，其属性 `num_attempts` 设置为 3，表示最多尝试 3 次。重试节点的子节点是 `ConnectToServer` 任务。

当行为树执行时，它首先进入重试节点。重试节点将执行其子节点 `ConnectToServer`。如果 `ConnectToServer` 返回成功，重试节点也返回成功，并继续执行序列节点的下一个子节点 `PerformTask`。如果 `ConnectToServer` 返回失败，重试节点将尝试重新执行 `ConnectToServer`，直到成功或达到最大重试次数（3 次）。如果在 3 次尝试后 `ConnectToServer` 仍然失败，重试节点将返回失败，导致整个序列节点也返回失败。

请注意，这个示例仅用于说明如何在行为树中使用重试节点。实际的行为树库可能有不同的语法和实现。在实际使用时，请参考您正在使用的行为树库的文档。

## 请问重试节点还有哪些属性可以设置？

重试节点的可设置属性可能因不同的行为树库而异。一般来说，重试节点至少具有以下属性：

1. **num_attempts**：尝试执行子节点的最大次数。当达到此次数且子节点仍返回失败时，重试节点将返回失败。

某些行为树库可能还提供其他可设置的属性，例如：

2. **delay_between_attempts**：在连续的重试尝试之间等待的时间（以秒或其他时间单位为单位）。这可以避免在短时间内连续尝试，给系统留出恢复的时间。

3. **timeout**：重试节点的最大执行时间（以秒或其他时间单位为单位）。当达到此时间限制时，重试节点将返回失败，不再尝试执行子节点。

请注意，这些属性可能因您正在使用的行为树库而有所不同。在实际使用时，请参考您正在使用的行为树库的文档，了解库中提供的属性和相关功能。
