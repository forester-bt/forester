# Runtime Engine Overview

The Forester runtime is the execution core that compiles `.tree` files, builds the behavior tree node graph, and drives the tick-by-tick evaluation loop.

It is built in Rust and is designed for both high-throughput robotics control loops and AI agent orchestration workflows.

---

## Core Subsystems

| Component | Description |
|---|---|
| **Engine (`Forester`)** | Orchestrates the tick loop, manages node state transitions, and schedules sync/async action execution. |
| **Blackboard** | In-memory shared key-value store for passing state between tree nodes across ticks. |
| **ActionKeeper** | Registry that maps action names declared with `impl` in `.tree` files to Rust handlers or remote action clients. |

---

## Execution Model

- **Synchronous by default**: The engine drives a deterministic tick loop. Synchronous actions block until they return `Success`, `Failure`, or `Running`.
- **Async environment**: Async actions return `Running` immediately and are polled on subsequent ticks without blocking the tick loop.
- **Parallelism**: `parallel` nodes tick multiple children in a single pass; async actions allow true concurrent I/O within a tick cycle.

---

## Entry Point: `ForesterBuilder`

The `ForesterBuilder` is the primary API for constructing and configuring a Forester runtime instance. It provides a safe, fluent interface for:

- Pointing to `.tree` source files or inline scripts
- Registering action implementations
- Pre-loading Blackboard state from JSON
- Enabling execution tracing

```rust
use forester_rs::runtime::builder::ForesterBuilder;
use forester_rs::runtime::action::{Action, ActionArgs, TickResult};
use forester_rs::runtime::action::builtin::data::StoreData;
use forester_rs::tracer::Tracer;

fn main() {
    let mut fb = ForesterBuilder::from_file_system();
    
    // Point to the tree entry file
    fb.main_file("main.tree".to_string());
    
    // Register built-in and custom actions
    fb.register_action("store", Action::sync(StoreData));
    fb.register_sync_action("my_action", MyAction);
    
    // Enable execution tracing
    fb.tracer(Tracer::default());
    
    // Pre-load Blackboard initial state from JSON
    fb.bb_load("db/initial_state.json".to_string());
    
    // Build and run
    let mut forester = fb.build().unwrap();
    let result = forester.run().unwrap();
    
    println!("Execution result: {:?}", result);
}
```

---

## Detailed Subsystem Documentation

- [Engine internals](./engine.md) — tick loop, node lifecycle, and state machine details
- [Runtime arguments](./rt_args.md) — configuring engine behavior at startup
- [Blackboard](./bb.md) — shared memory API and locking semantics
- [Actions](./r_actions.md) — registering sync, async, and remote actions
- [Trimming](./trimming.md) — modifying the running tree on the fly
- [Daemons](./daemons.md) — background tasks running alongside the tree