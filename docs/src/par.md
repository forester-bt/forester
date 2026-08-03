# Parallel Nodes

A **Parallel** node provides concurrent execution of its child nodes within a single engine tick. Unlike `sequence` or `fallback` nodes (which short-circuit immediately upon receiving a result), a `parallel` node ticks **all active children** during every tick pass.

In the Forester DSL, parallel nodes are declared using the `parallel` keyword.

---

## Execution Flow & Rules

1. **Concurrent Ticking**: During a tick pass, the parallel node iterates through all child nodes and ticks each one sequentially within that single frame.
2. **Child Processing**:
   - If a child returns `Success` or `Failure`, its status is stored.
   - If a child returns `Running`, it continues running. On subsequent ticks, already completed children (`Success` or `Failure`) are skipped, while `Running` children are re-ticked.
3. **Overall State Determination**:
   - **`Success`**: Returned when **all** child nodes evaluate to `Success`.
   - **`Failure`**: Returned if any child node returns `Failure` (unless a custom threshold policy is configured).
   - **`Running`**: Returned if at least one child node remains `Running` and no failure threshold has been breached.

---

## Code Example

```f-tree
import "std::actions"

root main sequence {
    // Run concurrent tasks in parallel
    parallel {
        clean_current_room()   // Async action (returns Running)
        inspect_environment()  // Sync or async action
    }
    
    navigate_to_next_room()
}

impl clean_current_room();
impl inspect_environment();
impl navigate_to_next_room();
```

---

## Key Considerations for Parallel Execution

1. **Async & Sync Actions**: Parallel nodes are ideal for launching multiple non-blocking async tasks simultaneously (e.g. streaming sensor data while running an LLM tool call).
2. **Non-Reactive Skipping**: Once a child completes with `Success` or `Failure`, the parallel node skips re-ticking it on subsequent frames until the parent parallel node finishes and resets.
3. **Shared Blackboard Access**: When children running in parallel write to the Blackboard, care should be taken to avoid key collisions or race conditions.
