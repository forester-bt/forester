# Export to ROS Nav2

[ROS](https://www.ros.org/) in general and [ROS Nav2](https://navigation.ros.org/) in particular are highly popular in robotics. They manage critical aspects of robot control, including navigation, localization, mapping, and more.

Forester provides native support for exporting a tree directly to ROS Nav2. The intermediate format is the [Nav2 XML format](https://navigation.ros.org/behavior_trees/index.html#).

The transformation process is straightforward.

## Control Nodes

Forester control nodes map directly to Nav2 control nodes:
- `sequence` becomes `PipelineSequence`
- `fallback` becomes `RoundRobin`
- `r_fallback` becomes `ReactiveFallback`

If a Forester control node is explicitly named, that name is applied to the resulting Nav2 XML element.

For example:
```forester
sequence FollowPathWithFallback {
    // ... 
}
```
Becomes:
```xml
<PipelineSequence name="FollowPathWithFallback">
</PipelineSequence>
```

## Actions

Forester actions map directly to Nav2 actions.

Every action accepts an implicit `name` parameter, which dictates the name of the action in the Nav2 tree. This parameter is optional and can be omitted.

For actions that take a subtree as a parameter (like decorators), the parameter must be named `sub`.

### Retry

Retry logic can be represented in two ways, depending on whether you need to name the node in Nav2:

- **The `retry` decorator:** The number of retries is specified compactly (e.g., `retry(3)`), but you cannot assign a name to the node.
- **The `RecoveryNode` action:** The number of retries is passed as an argument. This is the only way to explicitly assign a `name` parameter to the retry node.

```forester
    // Allows you to convey the name to Nav2
    RecoveryNode(
        number_of_retries = 1,
        name = "ComputePathToPose", 
        sub = ComputePathWithFallback()
    )
    
    // Cannot convey a name to Nav2
    retry(1) ComputePathWithFallback() 
    
    // Everything else functions exactly the same
```

## Example

**Forester Tree:**
```f-tree
import "ros::nav2"

root MainTree RecoveryNode(number_of_retries = 6, name = "NavigateRecovery", sub = NavigateWithReplanning())

sequence NavigateWithReplanning {
    RateController(
        hz = 1.0,
        sub = RecoveryNode(
            number_of_retries = 1,
            name = "ComputePathToPose",
            sub = retry(1) ComputePathWithFallback()
        )
    )
    retry(1) FollowPathWithFallback()
}

sequence ComputePathWithFallback {
    ComputePathToPose(goal = goal, path = path, planner_id = "GridBased")
    ComputePathToPoseRecoveryFallback()
}

sequence FollowPathWithFallback {
    FollowPath(path = path, controller_id = "FollowPath")
    FollowPathRecoveryFallback()
}

r_fallback ComputePathToPoseRecoveryFallback {
    GoalUpdated()
    ClearEntireCostmap(name = "ClearGlobalCostmap-Context", service_name = "global_costmap/clear_entirely_global_costmap")
}

r_fallback FollowPathRecoveryFallback {
    GoalUpdated()
    ClearEntireCostmap(name = "ClearLocalCostmap-Context", service_name = "local_costmap/clear_entirely_local_costmap")
}
```

**Transformed Nav2 XML:**
```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode name="NavigateRecovery" number_of_retries="6">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1">
          <RecoveryNode name="ComputePathToPose" number_of_retries="1">
            <RecoveryNode number_of_retries="1">
              <PipelineSequence name="ComputePathWithFallback">
                <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                <ReactiveFallback name="ComputePathToPoseRecoveryFallback">
                  <GoalUpdated/>
                  <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
                </ReactiveFallback>
              </PipelineSequence>
            </RecoveryNode>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1">
          <PipelineSequence name="FollowPathWithFallback">
            <FollowPath controller_id="FollowPath" path="{path}"/>
            <ReactiveFallback name="FollowPathRecoveryFallback">
              <GoalUpdated/>
              <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
            </ReactiveFallback>
          </PipelineSequence>
        </RecoveryNode>
      </PipelineSequence>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

## Tools

Ensure you are running the latest version of the CLI to use these features:
```shell
cargo install f-tree 
```

### Headers
To use Nav2 actions with the correct signatures, you must import the `ros::nav2` module in your Forester project.

To view the contents and available actions in this file, run:
```shell 
f-tree -d print-ros-nav2
```

### Exporting via the Console
To export the tree from the command line, run:
```shell
f-tree nav2 
```

### Exporting via the IntelliJ Plugin
Run the task `Export to ROS Nav2` directly within your IDE.

### Exporting via Code
```rust
#[test]
fn smoke() {
    let mut root_path = test_folder("ros/nav/smoke");

    let project = Project::build("main.tree".to_string(), root_path.clone()).unwrap();
    let tree = RuntimeTree::build(project).unwrap().tree;

    // Define the output file
    root_path.push("test.xml");

    // Export the tree
    tree.to_ros_nav(root_path).unwrap();
}
```