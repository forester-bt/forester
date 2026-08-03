# Trimming (Live Tree Modification)

**Trimming** is Forester's mechanism for modifying a running behavior tree at runtime — replacing, removing, or substituting nodes in the live execution graph without stopping the engine.

---

## Use Cases

### Performance & JIT Optimization
Cache or fold static subtrees that have already completed, reducing redundant evaluation on subsequent ticks. Analogous to JIT compiler transformations in virtual machines.

### Adaptive Logic
Dynamically swap decision branches based on runtime signals — Blackboard state, sensor readings, or LLM responses. Enables online reinforcement learning workflows where policy branches update as the tree runs.

### Research & Experimentation
Replace specific nodes mid-execution to compare behavioral outcomes under the same environmental conditions without restarting the engine.

---

## Core Components

### 1. `TrimTask`

A `TrimTask` encapsulates a single runtime modification. When registered with the engine, it is invoked on each tick with a snapshot of the current tree state. The task decides whether to apply, skip, or reject itself:

```rust
pub enum TrimTask {
    RtTree(Box<dyn RtTreeTrimTask>),
}
```

Register a trim task before running the tree:

```rust
forester.add_trim_task(TrimTask::rt_tree(MyTrimTask));
forester.run_until(Some(100)).unwrap();
```

---

### 2. `TrimRequest` — Task Decision States

Each invocation of `RtTreeTrimTask::process()` returns a `TrimRequest` indicating how the engine should proceed:

```rust
pub enum TrimRequest {
    Reject, // Permanently cancel this task (another task already made the change,
            // or the tree is no longer a valid target)
    Skip,   // Defer to the next tick (conditions not yet met)
    Attempt(RequestBody), // Proceed with the modification
}
```

| State | Description |
|---|---|
| **`Reject`** | Permanently cancels this task. Use when the modification is no longer valid or applicable. |
| **`Skip`** | Defers the task to the next tick. Use when waiting for a specific Blackboard value, tick number, or tree state. |
| **`Attempt(body)`** | Submits the modification for validation and application by the engine. |

---

### 3. Validation

Before applying a `TrimRequest::Attempt`, the engine validates that the nodes targeted for replacement are **not currently in a `Running` state**. If they are, the attempt is automatically deferred.

---

## Constraints & Best Practices

> **No ordering guarantees**: There is no guarantee of the order in which multiple trim tasks execute, or the exact tick at which any task will be applied.

- Design trim tasks to be **idempotent** — safe to apply multiple times without side effects.
- Always **validate the incoming tree snapshot** inside `process()` before constructing the `RequestBody`.
- Use `TrimRequest::Reject` when another task has already made the intended change.

---

## Example: Replace a Node After Tick 90

```rust
use forester_rs::runtime::trimmer::task::{RtTreeTrimTask, TrimTask};
use forester_rs::runtime::trimmer::{RequestBody, TreeSnapshot, TrimRequest};
use forester_rs::runtime::rtree::builder::{RtNodeBuilder, RtTreeBuilder};
use forester_rs::runtime::rtree::rnode::RNodeName;
use forester_rs::runtime::args::RtArgs;
use forester_rs::runtime::RtResult;

struct ReplaceFailWithSuccess;

impl RtTreeTrimTask for ReplaceFailWithSuccess {
    fn process(&self, snapshot: TreeSnapshot<'_>) -> RtResult<TrimRequest> {
        // Wait until tick 90 before applying this modification
        if snapshot.tick < 90 {
            return Ok(TrimRequest::Skip);
        }

        let tree = snapshot.tree;

        // Find the node named "fail_empty" in the running tree
        let target_id = tree
            .nodes
            .iter()
            .find(|(_, node)| {
                node.name()
                    .and_then(|n| n.name().ok())
                    .filter(|n| n.as_str() == "fail_empty")
                    .is_some()
            })
            .map(|(id, _)| id)
            .unwrap();

        // Build a replacement node: swap fail_empty -> success()
        let mut builder = RtTreeBuilder::new_from(tree.max_id() + 1);
        builder.set_as_root(action!(node_name!("success")), target_id.clone());

        Ok(TrimRequest::attempt(RequestBody::new(
            builder,
            Default::default(),
        )))
    }
}

fn run(mut forester: Forester) {
    forester.add_trim_task(TrimTask::rt_tree(ReplaceFailWithSuccess));
    let result = forester.run_until(Some(100)).unwrap();
    println!("Result: {}", result);
}
```