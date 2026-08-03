# Runtime Arguments (`RtValue`)

When a behavior tree action is ticked, its parameters are passed to the Rust handler as **runtime argument values** (`RtValue`). These are the runtime representations of the typed static arguments declared in the `.tree` DSL.

---

## The `RtValue` Enum

```rust
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum RtValue {
    String(String),
    Bool(bool),
    Number(RtValueNumber),
    Array(Vec<RtValue>),
    Object(HashMap<String, RtValue>),
    Pointer(BBKey),
}
```

### Primitive Types
- **`String`**, **`Bool`**, **`Number`**: Direct value types, equivalent to their counterparts in standard languages.

### Complex Types
- **`Array`**: An ordered list of `RtValue` elements.
- **`Object`**: A JSON-style key-value map (`HashMap<String, RtValue>`), useful for structured action configuration.

### Pointer (Blackboard Reference)
- **`Pointer(BBKey)`**: A live reference to a Blackboard key. When resolved, the action fetches the current value stored at that Blackboard cell rather than using a static literal.

---

## Pointer Resolution

A **Pointer** allows passing dynamic, live Blackboard state into an action without hardcoding the value at call time.

```f-tree
import "std::actions"

root main r_sequence {
    // Stores the current tick count into bb["tick"]
    store_tick("tick")
    
    // equal() receives a Pointer to bb["tick"], not the literal string "tick"
    r_fallback {
        equal(tick, 10)
        running()
    }
}
```

> **Note**: `store_tick("tick")` receives `"tick"` as a `String` (the cell name to write into). In contrast, `equal(tick, 10)` receives `tick` as a `Pointer` (meaning: resolve the value stored at Blackboard key `"tick"` and compare it to `10`).

### Pointer Indirection Example

Pointers can also point to cells that themselves contain key names (double indirection):

```f-tree
import "std::actions"

root main sequence {
    store("x", "tick")      // bb["x"] = "tick"
    store_tick(x)           // x is a Pointer -> writes to bb["x"] which is tick and resolves to bb["tick"]
    equal(tick, 10)         // tick is a Pointer -> reads bb["tick"] and compares to 10
}
```

---

## Extracting Argument Values in Rust

Forester provides two methods for reading action arguments in Rust action implementations:

### 1. Direct Cast: `as_<type>()`

The fastest approach — directly converts the `RtValue` to a primitive or complex type. Does **not** resolve Pointers, so use only when you are certain no Blackboard references will be passed:

```rust
fn handle(v: RtValue) {
    let val: Option<String> = v.as_string();
    let num: Option<f64>    = v.as_float();
}
```

### 2. Context-Aware Cast: `cast(ctx)`

Resolves Pointers by looking up the referenced Blackboard key at invocation time. Use this when your action may receive either a literal or a Blackboard reference:

```rust
use forester_rs::runtime::action::{Impl, RtArgs, Tick};
use forester_rs::runtime::context::TreeContextRef;
use forester_rs::runtime::RuntimeError;

impl Impl for CheckEqual {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let key = args
            .find_or_ith("key".to_string(), 0)
            .ok_or(RuntimeError::fail("the 'key' argument is required".to_string()))?;
        
        // cast(ctx) resolves Pointer values against the live Blackboard
        let resolved: String = key.cast(ctx.clone()).str()?;
        
        Ok(TickResult::success())
    }
}
```

> **Recommendation**: Prefer `cast(ctx)` in most action implementations. It handles both literal values and Blackboard pointers transparently.