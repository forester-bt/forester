# Invocations & Higher-Order Composition

In Forester, an **Invocation** calls a declared sub-tree definition, built-in decorator, action, or inline lambda within another tree definition.

Invocations are the primary mechanism for composing complex behavior trees out of smaller, modular building blocks.

---

## Basic Invocation Syntax

Invocations support both **positional** and **named** argument syntax:

```f-tree
import "std::actions"

// Sub-tree definition with parameters
sequence check_distance(item: object, threshold: number) {
    store("target_item", item)
    handle_distance(item, threshold)
}

root main sequence {
    // Positional argument invocation
    check_distance({"x": 10, "y": 20}, 50)
    
    // Named argument invocation
    check_distance(
        item = {"x": 30, "y": 40},
        threshold = 100
    )
}

impl handle_distance(item: object, limit: number);
```

---

## Functional Composition Capabilities

Forester extends standard behavior tree semantics by offering advanced functional composition primitives:

### 1. Higher-Order Trees (HOT)
Pass sub-trees as parameters (`t: tree`) to higher-order tree definitions and delegate execution using the `t(..)` syntax. This allows developers to write generic retry, fallback, or logging wrappers once and reuse them across the codebase.

*Read more in [Higher-Order Trees](./hot.md).*

---

### 2. Lambdas (Anonymous Inline Sub-Trees)
Define and instantly invoke inline, anonymous sub-trees (`lambda sequence { ... }`) without creating throwaway named definitions.

*Read more in [Lambdas](./lambda.md).*

---

## Summary of Invocation Types

| Invocation Type | Example Syntax | Description |
|---|---|---|
| **Standard Sub-Tree** | `navigate_to(target)` | Invokes a named sub-tree definition. |
| **Action Invocation** | `call_llm(prompt)` | Invokes an external `impl` action. |
| **Delegated Tree (HOT)** | `sub_action(..)` | Invokes a sub-tree passed as a parameter. |
| **Inline Lambda** | `lambda sequence { ... }` | Creates and executes an anonymous sub-tree inline. |