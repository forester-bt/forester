# Tree Definitions Overview

A **Tree Definition** declares a reusable sub-tree or node contract in Forester. Definitions isolate control logic into modular components that can accept typed parameters, store state on the Blackboard, or wrap child trees.

Forester classifies tree definitions into four main categories:

---

## Categories of Definitions

### 1. Control Flow Nodes
Control flow nodes govern how child nodes are scheduled and executed:
* **`sequence`**: Ticks children sequentially until one returns `Failure` or `Running`. Returns `Success` if all children succeed.
* **`fallback`**: Ticks children sequentially until one returns `Success` or `Running`. Returns `Failure` if all children fail (often used for fallbacks and recovery).
* **`parallel`**: Ticks children concurrently based on a specified synchronization policy (e.g. success threshold).

*Read more in [Control Flow Nodes](./flow.md).*

---

### 2. Root Entry Point (`root`)
The `root` definition declares the top-level starting point of the behavior tree. A Forester script must contain at least one `root` definition:

```f-tree
root main sequence {
    check_preconditions()
    execute_agent_task()
}
```

---

### 3. Decorator Nodes
Decorators are single-child nodes that modify or control the execution behavior of their child:
* **`retry(N)`**: Automatically re-ticks its child up to **N** times upon failure.
* **`inverter`**: Flips child `Success` to `Failure` and vice versa.
* **`timeout(ms)`**: Limits child execution time.
* **`repeat(N)`**: Executes child **N** times sequentially.

*Read more in [Decorators](./decorators.md).*

---

### 4. Action Nodes (`impl`)
Actions represent the leaves of the behavior tree where actual work gets executed (e.g. motor movement, REST calls, LLM prompt generation). Action contracts are defined using `impl`:

```f-tree
impl call_llm_tool(prompt: string, model: string);
impl execute_motor_step(velocity: number);
```

*Read more in [Actions](./actions.md).*

---

### 5. Lambdas & Higher-Order Trees
* **Higher-Order Trees (HOT)**: Definitions that accept other sub-trees as parameters (`sub_tree: tree`).
* **Lambdas**: Anonymous inline sub-tree definitions created and executed at the point of invocation.

*Read more in [Higher-Order Trees & Lambdas](./invoc_intro.md).*