# Execution Tracing

The Forester tracer records a tick-by-tick log of node evaluations as the behavior tree runs. It captures node IDs, tick results, node state parameters, and any custom messages emitted by actions — making it the primary tool for debugging and post-execution analysis.

---

## Reading Trace Output

Each trace line follows the format:

```
[tick]  node_id : Status(parameters...)
```

- **`[tick]`**: The current tick number.
- **Indent**: Reflects nesting depth in the tree.
- **`node_id`**: Unique integer ID of the node being evaluated.
- **`Status(...)`**: Result state (`Running`, `Success`, `Failure`) with relevant parameters (cursor position, child count, key-value pairs, etc.).

### Example Trace

```text
[1]  1 : Running(cursor=0,len=1)
[1]    2 : Running(cursor=0,len=3)
[1]      3 : Success(key=x,value=tick)
[1]    2 : Running(cursor=1,len=3)
[1]      4 : Success(name=tick)
[1]    2 : Running(cursor=2,len=3)
[1]      5 : Running(cursor=0,len=2)
[1]        6 : Success(k=a,i=1)
[1]      5 : Running(cursor=1,len=2)
[1]        7 : Running(cursor=0,len=2)
[1]          8 : Failure(key=x,expected=10,reason=1 != 10)
[1]        7 : Running(cursor=1,len=2)
[1]          9 : Running()
[2]  next tick
[2]    2 : Running(cursor=0,len=3)
[2]      3 : Success(key=x,value=tick)
...
```

---

## Enabling the Tracer

Enable tracing in `ForesterBuilder` before running the engine:

```rust
use forester_rs::tracer::{Tracer, TracerConfiguration};
use forester_rs::runtime::builder::ForesterBuilder;

let mut fb = ForesterBuilder::from_file_system();
fb.main_file("main.tree".to_string());

// Default tracer (stdout output)
fb.tracer(Tracer::default());

// Configured tracer (output to file, custom indent)
fb.tracer(Tracer::create(TracerConfiguration {
    indent: 2,
    to_file: Some("output/main.trace".to_string()),
    time_format: None,
}));
```

### Configuration Options

| Option | Type | Description |
|---|---|---|
| `indent` | `usize` | Number of spaces per nesting level in the trace output. |
| `to_file` | `Option<String>` | If set, writes the trace to the specified file path instead of stdout. |
| `time_format` | `Option<String>` | If set, prepends a formatted timestamp to each trace line. |

---

## Custom Trace Messages

Actions can emit custom messages inline within the trace output using `ctx.trace()`. This is useful for logging intermediate Blackboard values or action-specific diagnostic information:

```f-tree
import "std::actions"

impl tracked_action();

root main repeat(3) {
    tracked_action()
}
```

```rust
use forester_rs::runtime::action::{Impl, RtArgs, Tick, TickResult};
use forester_rs::runtime::context::TreeContextRef;

struct TrackedAction;

impl Impl for TrackedAction {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let mut ctx = ctx.lock()?;
        
        // Read current counter from Blackboard
        let i = ctx.bb()
            .get("counter".to_string())?
            .and_then(|v| v.clone().as_int())
            .map(|v| v + 1)
            .unwrap_or(0);
        
        ctx.bb().put("counter".to_string(), RtValue::int(i))?;
        
        // Emit a custom trace message
        ctx.trace(format!("counter = {:?}", i));
        
        Ok(TickResult::success())
    }
}
```

This produces inline messages in the trace output:

```text
[1]    2 : Running(len=1)
[1]      counter = 0
[1]      3 : Success()
[2]  next tick
[2]      counter = 1
[2]      3 : Success()
[3]  next tick
[3]      counter = 2
[3]      3 : Success()
[3]  1 : Success(cursor=0,len=1)
```

---

## Accessing the Tracer via HTTP

When the engine HTTP server is running, the tracer output can be fetched or appended to at runtime:

```shell
# Print current trace
curl http://localhost:10000/tracer/print

# Append a custom event to the trace
curl -X POST http://localhost:10000/tracer/custom \
  -H "Content-Type: application/json" \
  -d '{"message": "external checkpoint reached"}'
```