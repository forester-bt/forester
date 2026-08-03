# Engine Internals

The `Forester` engine is the central execution runtime. It drives the tick loop, manages node state, dispatches actions, and coordinates the Blackboard, ActionKeeper, and optional tracer.

---

## Internal Components

| Component | Description |
|---|---|
| **Engine Core** | Manages the tree node graph, drives the tick loop, and handles state transitions (`Running`, `Success`, `Failure`). |
| **Blackboard** | In-memory shared key-value store for passing state between nodes. |
| **ActionKeeper** | Registry mapping `impl` action names to Rust handlers or remote action clients. |
| **Tracer** | Optional execution logger that records tick-by-tick node evaluations for debugging and replay. |

---

## Tick Loop

On each tick, the engine traverses the tree from the root node down, evaluating each node according to its type (sequence, fallback, parallel, decorator, or action). The tick loop continues until the root node returns `Success` or `Failure`.

### Bounded Execution

To prevent infinite loops and enable time-boxed execution (useful for simulation or testing), the tick count can be capped:

```rust
// Run for a maximum of 100 ticks
forester.run_until(Some(100)).unwrap();

// Run until the root returns Success or Failure (unbounded)
forester.run().unwrap();
```

---

## Runtime Environment

The engine uses [Tokio](https://tokio.rs/) as its async runtime to schedule and parallelize async action execution.

By default, the engine creates its own Tokio runtime. If the engine is embedded in an application that already has a Tokio runtime, it can be provided via `ForesterBuilder`:

```rust
use forester_rs::runtime::builder::ForesterBuilder;
use forester_rs::tracer::Tracer;
use forester_rs::runtime::action::Action;
use forester_rs::runtime::action::builtin::data::StoreData;

fn main() {
    let mut fb = ForesterBuilder::from_file_system();
    fb.main_file("main.tree".to_string());
    fb.register_action("store", Action::sync(StoreData));
    fb.tracer(Tracer::default());
    fb.bb_load("db/initial_state.json".to_string());

    let mut forester = fb.build().unwrap();

    // Unbounded run
    let result = forester.run().unwrap();
    println!("Result: {:?}", result);
}
```

---

## Built-In HTTP Server

The engine optionally exposes an HTTP server during tree execution. This enables external processes — including remote action servers and monitoring tools — to interact with the Blackboard and tracer at runtime.

Enable it by specifying a port in `ForesterBuilder`:

```rust
fb.http_serv(10000);
```

> **Note**: The HTTP server shuts down automatically when the root tree finishes execution.

### HTTP API Endpoints

| Method | Endpoint | Description |
|---|---|---|
| `GET` | `/` | Health check. Returns `Ok`. |
| `GET` | `/tracer/print` | Prints current tracer output. |
| `POST` | `/tracer/custom` | Appends a custom event to the tracer log (body: `CustomEvent` JSON). |
| `GET` | `/bb/:key` | Reads value at `key` from the Blackboard. |
| `POST` | `/bb/:key` | Writes a value to `key` in the Blackboard (body: `RtValue` JSON). |
| `GET` | `/bb/:key/take` | Reads and removes `key` from the Blackboard. |
| `GET` | `/bb/:key/lock` | Locks `key` to prevent concurrent writes. |
| `GET` | `/bb/:key/unlock` | Unlocks `key`. |
| `GET` | `/bb/:key/locked` | Returns whether `key` is currently locked. |
| `GET` | `/bb/:key/contains` | Returns whether `key` exists in the Blackboard. |