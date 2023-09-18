# Console f-tree

The console utility `f-tree` can be installed using `cargo` and can be used to simulate and visualize the tree.

The [Intellij plugin](intellij.md) basically wraps this utility and provides the same functionality.

```shell
cargo install f-tree
```

and then be used with

```shell
~ f-tree --help
Usage: f-tree [OPTIONS] <COMMAND>

Commands:
  print-std-actions  Print the list of std actions from 'import std::actions'
  print-ros-nav2     Print the list of ros actions from 'import ros::nav2'
  sim                Runs simulation. Expects a simulation profile
  vis                Runs visualization. Output is in svg format.
  nav2               Convert to the xml compatable format of nav ros2.
  help               Print this message or the help of the given subcommand(s)

Options:
  -d, --debug    Print debug logs
  -h, --help     Print help
  -V, --version  Print version


```
