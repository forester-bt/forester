# Tools

The Forester ecosystem includes several auxiliary tools and libraries to help you write, analyze, and execute behavior trees.

## IntelliJ Plugin

The [Forester IntelliJ Plugin](https://github.com/forester-bt/forester-intellij-plugin) provides native IDE support for the `.tree` language. Features include:
* Syntax highlighting
* Code folding
* Code navigation
* Code formatting
* Code inspections and error highlighting
* Structure view
* Built-in tasks to visualize and simulate the tree directly from the IDE

## Remote Action Libraries

Remote action clients allow you to decouple the orchestration engine from the execution of individual actions. This is particularly useful for AI agents where the orchestrator is in Rust, but the tool execution is in Python. Features include:
* The ability to execute specific actions on a remote machine or in a separate process via HTTP, returning the `TickResult` to the Forester runtime.
* Bidirectional access, allowing the remote action to read from and write to the shared Blackboard.

**Available Clients:**
* [Python Client (`forester-http-ra-py`)](https://github.com/forester-bt/forester-http-ra-py)
* [Rust Client (`forester-rs`)](https://github.com/forester-bt/forester-rs)

## CLI (`f-tree`)

The `f-tree` command-line interface is the primary utility for testing and manipulating trees without writing boilerplate application code. It supports:
* Simulating trees with stubbed actions using YAML profiles.
* Generating SVG visualizations of the tree structure.
* Exporting trees to the ROS Nav2 XML format.
* Printing built-in standard library headers (e.g., `ros::nav2`, `std::actions`).