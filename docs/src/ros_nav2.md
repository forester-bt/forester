# Export to ROS Nav2
[ROS](https://www.ros.org/) in general and [ROS Nav2](https://navigation.ros.org/) in particular are very popular in robotics.
They take care of many aspects of robot control, including navigation, localization, mapping, and more.

Forester provides a way to export a tree to ROS Nav2.
The intermediate format is [Nav2](https://navigation.ros.org/behavior_trees/index.html#) XML format.

The transformation format for now is pretty straightforward.

## Control nodes
The control nodes are mapped to the nav2 control nodes directly:
 - `PipelineSequence` to `sequence`
 - `RoundRobin` to `fallback`
 - `ReactiveFallback` to `r_fallback`

If the control node has a name, it is used as the name of the control node in the nav2 tree.

```
sequence FollowPathWithFallback{
    .. 
}
```
will become 
```xml
<PipelineSequence name="FollowPathWithFallback">
</PipelineSequence>
```

## Actions
The actions can be mapped straightforwardly to the nav2 actions.

Every action can implicitly take a `name` parameter, which is used as the name of the action in the nav2 tree. 
But the name parameter can be omitted also.

Some of the actions take the subtree as a parameter. The parameter has a name `sub`

### Retry
Retry is represented in two options:
 - `Retry` - the number of retries is specified  without a name like `retry(3)`
 - `RecoveryNode` - the number of retries is specified with the default way like `RecoveryNode(number_of_retries = 3)`. 
This becomes only way to convey the name of the node.

```f-tree
    RecoveryNode(
            number_of_retries = 1,
            name = "ComputePathToPose", // it allows to convey the name
            sub = ComputePathWithFallback()
        )
    retry(1) ComputePathWithFallback() // it is not possible to convey the name
    
    // but everything else is the same    
```

## Example

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
sequence ComputePathWithFallback{
    ComputePathToPose(goal = goal,path = path,planner_id = "GridBased")
    ComputePathToPoseRecoveryFallback()
}

sequence FollowPathWithFallback{
    FollowPath(path = path,controller_id = "FollowPath")
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

will be transformed into

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
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
            <FollowPath path="{path}" controller_id="FollowPath"/>
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

The changes arrived in the latest version of f-tree, therefore better to update f-tree
```shell
cargo install f-tree 
```

### Headers
To have headers for nav2 actions, you need to import the `ros::nav2` module in your project.
To see the content of the file, run

```shell 
f-tree -d print-ros-nav2
```

### Export from console

To export the tree from the console, run

```shell
f-tree.exe nav2 
```

### Export from Intellij plugin

Run the task `Export to ROS Nav2`

### Export from code

```rust
#[test]
fn smoke() {
    let mut root_path = test_folder("ros/nav/smoke");

    let project = Project::build("main.tree".to_string(), root_path.clone()).unwrap();
    let tree = RuntimeTree::build(project).unwrap().tree;
    fb.push("test.xml");
    
    tree.to_ros_nav(root_path.clone()).unwrap();

}
```