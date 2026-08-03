# Forester Syntax Overview

The syntax of the `.tree` language is declarative, strongly typed, and modular. A Forester script consists of six primary elements:

1. **Imports**: Module references to include external `.tree` files or built-in stdlib actions.
2. **Root Declarations**: The entry point of the behavior tree (`root main ...`).
3. **Tree Definitions**: Named sub-trees (`sequence`, `fallback`, `parallel`) with typed parameters.
4. **Action Implementations (`impl`)**: Contract definitions for native or remote actions executed by the runtime.
5. **Invocations & Decorators**: Calls to sub-trees and actions, wrapped with built-in decorators (e.g. `retry`, `inverter`, `timeout`).
6. **Higher-Order Tree Delegates (`..`)**: Invoking sub-trees passed as parameters.

---

## Comprehensive Syntax Example

```f-tree
// 1. Imports
import "std::actions"
import "sensors/battery.tree" {
    check_battery => is_battery_ok,
}

// 2. Root Entry Point
root main sequence {
    // Check battery or charge
    fallback {
        is_battery_ok()
        charge_robot()
    }
    
    // Execute target action with retry decorator
    retry(3) execute_task(
        target = {"x": 10, "y": 20},
        sub_action = place_item([100])
    )
}

// 3. Higher-Order Tree Definition
sequence execute_task(target: object, sub_action: tree) {
    fallback {
        is_target_reachable(target)
        approach_target(target)
    }
    sequence {
        savepoint()
        sub_action(..) // Delegate invocation of passed sub_action
    }
}

// 4. Sub-tree Definition
sequence place_item(location: array) {
    validate_location(location)
    info_wrapper(drop_payload({"speed": "slow"}))
}

// 5. Decorator Wrapper Pattern
sequence info_wrapper(action: tree) {
    log("Starting action execution")
    action(..)
    log("Action execution completed")
}

// 6. Action Signatures (linking to runtime implementation)
impl charge_robot();
impl approach_target(pos: object);
impl validate_location(coords: array);
impl drop_payload(config: object);
impl log(message: string);
```

---

## Core Constructs Quick Reference

| Element | Example Syntax | Description |
|---|---|---|
| **Root Tree** | `root main sequence { ... }` | Mandatory entry point for execution. |
| **Sequence Node** | `sequence name(params) { ... }` | Executes children in order until one fails. |
| **Fallback Node** | `fallback name(params) { ... }` | Executes children in order until one succeeds. |
| **Action Definition** | `impl action_name(arg: type);` | Defines an action implemented in Rust/Python. |
| **Tree Parameter** | `sub_tree: tree` | Pass a sub-tree as a parameter (Higher-Order Tree). |
| **Tree Delegate** | `sub_tree(..)` | Invoke a passed sub-tree parameter. |
| **Decorator** | `retry(3) node(...)` | Wraps execution with retry/inverter/timeout logic. |
| **Blackboard Pointer**| `&key_name` | Pass live reference to shared Blackboard memory. |
