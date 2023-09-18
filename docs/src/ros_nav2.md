# Export to ROS Nav2
[ROS](https://www.ros.org/) in general and [ROS Nav2](https://navigation.ros.org/) in particular are very popular in robotics.
They take care of many aspects of robot control, including navigation, localization, mapping, and more.

Forester provides a way to export a tree to ROS Nav2.
The intermediate format is [Nav2](https://navigation.ros.org/behavior_trees/index.html#) XML format.

The transformation format for now is pretty straightforward.

To have headers for nav2 actions, you need to import the `ros::nav2` module in your project.
To see the content of the file, run 

```shell 
f-tree -d print-ros-nav2
```

## Actions
The actions can be mapped straightforwardly to the nav2 actions.

Every action can implicitly take a `name` parameter, which is used as the name of the action in the nav2 tree. 
But the name parameter can be omitted.
Some of the actions take the subtree as a parameter. The parameter has a name `sub`

