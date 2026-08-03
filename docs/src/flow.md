# Control Flow Nodes Overview

Control flow nodes direct the execution path of a behavior tree. They manage how child nodes are scheduled, evaluated, and short-circuited based on child tick results (`Success`, `Failure`, or `Running`).

Forester supports three fundamental control flow nodes:

---

## 1. Sequence Nodes (`sequence`)
* **Behavior**: Evaluates children sequentially from left to right.
* **Short-circuiting**: 
  - Returns `Failure` immediately if any child returns `Failure`.
  - Returns `Running` if a child returns `Running` (halting further evaluation during that tick).
  - Returns `Success` only when **all** children return `Success`.
* **Use Case**: Step-by-step procedures (e.g. `[Check Precondition -> Execute Step 1 -> Execute Step 2]`).

*Detailed documentation: [Sequence Nodes](./seq.md).*

---

## 2. Fallback / Selector Nodes (`fallback`)
* **Behavior**: Evaluates children sequentially from left to right to find a successful path.
* **Short-circuiting**:
  - Returns `Success` immediately if any child returns `Success`.
  - Returns `Running` if a child returns `Running`.
  - Returns `Failure` only when **all** children return `Failure`.
* **Use Case**: Error handling, recovery strategies, and fallback routines (e.g. `[Primary Action -> Secondary Fallback Action -> Alert Operator]`).

*Detailed documentation: [Fallback Nodes](./falls.md).*

---

## 3. Parallel Nodes (`parallel`)
* **Behavior**: Ticks children concurrently during each engine tick.
* **Completion Policy**: Evaluates overall status based on child results and configured threshold policies (e.g., success threshold or fail-fast rules).
* **Use Case**: Concurrent tasks (e.g. `[Maintain Balance while Navigating to Goal]`).

*Detailed documentation: [Parallel Nodes](./par.md).*

---

## Summary of Return States

| Node Type | Succeeds When | Fails When | Short-Circuits On |
|---|---|---|---|
| **`sequence`** | All children return `Success` | Any child returns `Failure` | First `Failure` or `Running` |
| **`fallback`** | Any child returns `Success` | All children return `Failure` | First `Success` or `Running` |
| **`parallel`** | Reaches success threshold | Reaches failure threshold | Policy dependant |
