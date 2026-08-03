# Runtime Actions

Actions are the leaf nodes of the behavior tree — the points where actual work gets done. Every action declared with `impl` in a `.tree` file must be bound to a Rust handler (or a remote client) via `ForesterBuilder` before the tree can run.

---

## Action Types

Forester supports three execution modes for actions:

| Type | Blocking? | Use Case |
|---|---|---|
| **Sync** | Yes — blocks the tick until done | Fast, CPU-bound, or deterministic operations |
| **Async** | No — returns `Running` immediately | I/O-bound operations, LLM calls, network requests |
| **Remote** | Yes — sends an HTTP request and waits | External processes (Python agents, microservices) |

> For heavy or slow operations, prefer **async actions** to avoid blocking the tick loop.

---

## Action Traits

### 1. Sync Actions (`Impl`)

Sync actions block the tick loop until they return a result. They are the only action type that currently supports the `halt()` callback:

```rust
use forester_rs::runtime::action::{RtArgs, Tick, RtOk};
use forester_rs::runtime::context::TreeContextRef;

pub trait Impl {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick;

    // Called when a reactive flow node (r_sequence, r_fallback) preempts this action.
    // Implement this to clean up resources gracefully.
    // Default is a no-op.
    fn halt(&self, args: RtArgs, ctx: TreeContextRef) -> RtOk {
        Ok(())
    }
}
```

> **Important**: `halt()` must return as quickly as possible — it must not block execution.

---

### 2. Async Actions (`ImplAsync`)

Async actions are spawned in a separate Tokio task and return `Running` on the first tick. The engine re-polls them on subsequent ticks until they resolve:

```rust
use forester_rs::runtime::action::{RtArgs, Tick};
use forester_rs::runtime::context::TreeContextRef;

pub trait ImplAsync: Sync + Send {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick;
}
```

> **Note**: When the engine runs with a tick limit (`run_until(Some(N))`), async actions consume ticks while `Running`. Account for this when setting tick budgets.

---

### 3. Remote Actions (`ImplRemote`)

Remote actions send an HTTP POST request to an external server and wait for the response. This is the mechanism for Python AI tool integration via `forester-http-ra-py`.

```rust
pub trait ImplRemote: Sync + Send {
    fn tick(&self, args: RtArgs, ctx: TreeRemoteContextRef) -> Tick;
}

pub struct TreeRemoteContextRef<'a> {
    pub curr_ts: Timestamp, // Current tick timestamp
    pub port: u16,          // Port of the engine HTTP server (for Blackboard access)
    pub env: &'a mut RtEnv, // Runtime env for making HTTP requests
}
```

The default remote action implementation (`RemoteHttpAction`) handles the HTTP protocol automatically:

```rust
use forester_rs::runtime::action::builtin::remote::RemoteHttpAction;

// Register a remote action pointing to an external Python action server
fb.register_remote_action(
    "call_llm_tool",
    RemoteHttpAction::new("http://localhost:9000/call_llm", None)
);
```

The engine sends each tick as a `RemoteActionRequest`:

```rust
pub struct RemoteActionRequest {
    pub tick: usize,           // Current tick number
    pub args: Vec<RtArgument>, // Action arguments from the tree
    pub serv_url: String,      // Engine HTTP server URL (for Blackboard access)
}
```

The remote server responds with a `TickResult` (`Success`, `Failure`, or `Running`).

*See [Remote Action Clients](./rem_action.md) for how to implement the server side in Python or Rust.*

---

## Registering Actions

Register sync, async, and remote actions with `ForesterBuilder` before running the tree:

```rust
use forester_rs::runtime::builder::ForesterBuilder;
use forester_rs::runtime::action::Action;
use forester_rs::runtime::action::builtin::data::StoreData;
use forester_rs::runtime::action::builtin::remote::RemoteHttpAction;

let mut fb = ForesterBuilder::from_file_system();
fb.main_file("main.tree".to_string());

// Sync action (built-in)
fb.register_action("store", Action::sync(StoreData));

// Custom sync action
fb.register_sync_action("my_sensor_check", MySensorCheck);

// Async action
fb.register_async_action("fetch_data", FetchData);

// Remote action (Python server)
fb.register_remote_action(
    "call_llm",
    RemoteHttpAction::new("http://localhost:9000/call_llm", None)
);
```

---

## Action Statefulness

Actions are intentionally **stateless** — they cannot hold mutable state between ticks. Persist any inter-tick state on the Blackboard instead:

```rust
impl Impl for CounterAction {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let mut ctx = ctx.lock()?;
        
        // Read current count from Blackboard
        let count: i64 = ctx.bb().get("count")
            .and_then(|v| v.as_int())
            .unwrap_or(0);
        
        // Write incremented count back to Blackboard
        ctx.bb().put("count", RtValue::Number(RtValueNumber::Int(count + 1)))?;
        
        Ok(TickResult::success())
    }
}
```

---

## Built-In Actions

A set of ready-made action implementations for Blackboard operations and HTTP requests is available in:

```rust
use forester_rs::runtime::action::builtin::*;
```
