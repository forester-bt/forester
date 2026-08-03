# Actions (`impl`)

**Actions** form the leaf nodes of a behavior tree. They represent the actual work performed by external code—such as controlling hardware motors, querying databases, making REST calls, or invoking LLM tools.

In the Forester DSL, actions are declared using the `impl` keyword.

---

## Action Signatures (`impl`)

An action contract defines the action name, input parameters, and expected parameter types:

```f-tree
// Action signature declarations
impl navigate_to_pose(target: object);
impl check_battery_level(min_voltage: number);
impl call_llm_tool(prompt: string, model: string);
impl reset_system_state(){}
```

* **Semicolons**: A trailing semicolon `;` or empty block `{}` is required after an `impl` declaration.

---

## Action Execution Modes

The Forester runtime supports two primary execution mechanisms:

### 1. Local Actions (Rust Engine)
Implemented directly in Rust using the `Action` trait and registered with `ForesterBuilder`:

```rust
use forester_rs::runtime::action::{Action, ActionArgs, TickResult};

struct NavigateToPose;

impl Action for NavigateToPose {
    fn tick(&self, args: ActionArgs) -> TickResult {
        // Perform navigation logic
        TickResult::Success
    }
}
```

* **Synchronous Actions**: Block until execution completes during the current tick.
* **Asynchronous Actions**: Return `TickResult::Running` immediately for non-blocking operations, completing on subsequent ticks.

---

### 2. Remote Actions (Python / Microservices)
Actions executed in external processes (e.g. Python AI agents using `forester-http-ra-py`). The Forester engine sends an RPC/HTTP request to the remote action server and handles the response.

---

## Strict Contract Enforcement

The Forester compiler enforces strict type checking and argument count matching between `impl` signatures and invocation sites:

```f-tree
impl calculate_route(destination: object, max_speed: number);

root main sequence {
    // ❌ COMPILE ERROR: Missing required 'max_speed' argument
    calculate_route(destination = {"x": 10, "y": 20})
}
```

---

## Passing Blackboard References (`&pointer`)

Actions can accept live Blackboard references using the `&` pointer prefix. This allows nodes to read dynamically updated state without hardcoding values at definition time:

```f-tree
impl analyze_sensor_data(data_ref: string);

root main sequence {
    // Pass live Blackboard reference &sensor_reading
    analyze_sensor_data(data_ref = &sensor_reading)
}
```