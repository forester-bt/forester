# Analysis & Observability

Forester provides a suite of tools for inspecting, debugging, and validating behavior trees before and during execution. These tools are critical for building confidence in complex orchestration logic before deploying to production hardware or live AI agent workflows.

---

## 1. Visualization

Generate a visual diagram of any `.tree` file's structure using Graphviz. Visualizing the tree makes it easy to review control flow, catch structural mistakes, and communicate logic to stakeholders.

```shell
f-tree vis --input main.tree --output tree_diagram.svg
```

Output is in **SVG format**, readable in any browser or vector graphics tool.

*See [Visualization](./viz.md) for full usage.*

---

## 2. Execution Tracing

Enable step-by-step logging of tick evaluations — recording which nodes were visited, what states they returned, and how Blackboard values changed across ticks.

Tracing is configured in `ForesterBuilder`:

```rust
use forester_rs::tracer::Tracer;

fb.tracer(Tracer::default());
```

The trace output can be printed to stdout, written to a file, or queried via the engine's HTTP API at `/tracer/print`.

*See [Tracing](./trace.md) for full usage.*

---

## 3. Simulation

Run the behavior tree with **stub action implementations** instead of real hardware or external API calls. The simulator lets you define per-action responses (`Success`, `Failure`, or `Running`) and observe how the tree navigates them — without any external dependencies.

```shell
f-tree sim --profile sim_profile.json
```

This is especially valuable for:
- **Robotics**: Validate control flow before connecting to hardware.
- **AI agents**: Test fallback chains and retry logic before incurring LLM API costs.

*See [Simulation](./sim.md) for full usage.*

---

## 4. ROS Nav2 Export

Export Forester trees to **ROS Nav2 XML format** for direct use in ROS2 navigation pipelines. This allows Forester-authored trees to be dropped into existing Nav2 workflows as a validated behavior plugin.

*See [ROS Nav2 Export](./ros_nav2.md) for full usage.*
