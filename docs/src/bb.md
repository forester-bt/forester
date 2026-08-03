# Blackboard

The **Blackboard** is Forester's shared in-memory state store — a key-value store that nodes read from and write to across ticks. It is the primary mechanism for passing data between actions and tree nodes without tight coupling.

In AI agent workflows, the Blackboard acts as the **agent's working context**: LLM responses, tool call results, and session state are written here and read by downstream nodes.

---

## Data Format

The Blackboard stores pairs of `String` keys and `BBValue` values:

```rust
#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub enum BBValue {
    Locked(RtValue),    // Key exists and is locked: read-only, no further writes
    Unlocked(RtValue),  // Key exists and is freely readable and writable
    Taken,              // Key slot exists but value has been consumed (taken)
}
```

### Key States

| State | Description |
|---|---|
| **`Unlocked(RtValue)`** | Normal state. The value can be read, overwritten, or taken by any node. |
| **`Locked(RtValue)`** | The value is protected from modification. Other nodes can still read it, but cannot overwrite or take it. |
| **`Taken`** | The key slot exists but its value has been consumed by a `take` operation. |

---

## Accessing the Blackboard from DSL

Use the built-in `std::actions` to interact with the Blackboard directly from `.tree` files:

```f-tree
import "std::actions"

root main sequence {
    // Write a value
    store("agent_mode", "active")
    
    // Lock a key to prevent modification during critical section
    lock("agent_mode")
    
    // Read and compare
    equal("agent_mode", "active")
    
    // Unlock when done
    unlock("agent_mode")
}
```

---

## Accessing the Blackboard from Rust

In Rust action implementations, the Blackboard is accessed through the `TreeContextRef`:

```rust
use forester_rs::runtime::context::TreeContextRef;
use forester_rs::runtime::action::{Impl, RtArgs, Tick, TickResult};

impl Impl for MyAction {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let mut ctx = ctx.lock()?;
        
        // Read a value from the Blackboard
        let mode = ctx.bb().get("agent_mode")?.and_then(|v| v.as_string());
        
        // Write a value to the Blackboard
        ctx.bb().put("result", RtValue::String("done".to_string()))?;
        
        Ok(TickResult::success())
    }
}
```

---

## Persistence: Load & Dump

The Blackboard supports JSON serialization for pre-loading initial state and dumping snapshots:

| Method | Description |
|---|---|
| `bb_load(path)` | Loads initial Blackboard state from a JSON file at engine startup (via `ForesterBuilder`). |
| `dump(path)` | Saves a snapshot of the full Blackboard to a JSON file. |
| `print_dump()` | Prints the current Blackboard snapshot to stdout in JSON format. |
| `text_dump()` | Returns the Blackboard snapshot as a JSON string. |

```rust
// Pre-load state at startup
fb.bb_load("db/initial_state.json".to_string());
```

---

## HTTP Access

When the engine's built-in HTTP server is enabled, the Blackboard is also accessible over HTTP at runtime. See the [Engine HTTP API](./engine.md#http-api-endpoints) for endpoint details.

---

## Utilities

A set of helper utilities for common Blackboard operations is available in the `blackboard::utils` module, for example:

- `blackboard::utils::push_to_arr(key, value, ctx)` — Appends a value to an array stored at a given key.