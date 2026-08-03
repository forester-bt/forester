# Higher-Order Trees (HOT)

A **Higher-Order Tree** is a tree definition that accepts other sub-trees as typed parameters (`param: tree`) and delegates execution to them using the `param(..)` syntax.

This is Forester's most powerful functional abstraction. It eliminates repetitive copy-pasted retry/fallback/logging patterns by letting you write a pattern once and inject the varying behavior as a parameter.

---

## Motivation

Consider a common robotics pattern: check a precondition, and if it fails, run a corrective task. Without HOTs you repeat this `fallback` structure for every step:

```f-tree
sequence handle(item: object) {
    fallback { close_enough(item)  approach(item) }
    fallback { is_graspable(item)  grasp(item) }
    fallback { enough_space(item)  sequence { move(item) save(item) } }
}
```

With a Higher-Order Tree you name the pattern once and inject the steps:

```f-tree
// Define the pattern once
fallback precond_or_fix(condition: tree, fix: tree) {
    condition(..)
    fix(..)
}

// Reuse it without duplication
sequence handle(item: object) {
    precond_or_fix(close_enough(item),  approach(item))
    precond_or_fix(is_graspable(item),  grasp(item))
    precond_or_fix(enough_space(item),  sequence { move(item) save(item) })
}
```

---

## Syntax

Declare a `tree`-typed parameter in the definition signature. Invoke it inside the body using `param_name(..)`:

```f-tree
// Generic retry-with-fallback wrapper
fallback retryer(action: tree, on_fail: tree) {
    retry(3) action(..)
    on_fail(..)
}

root main sequence {
    // Pass any tree as action or on_fail
    retryer(
        call_llm({"model": "gpt-4o"}),
        alert_operator()
    )
}

impl call_llm(config: object);
impl alert_operator();
```

---

## Lambdas as HOT Arguments

Inline lambdas can be passed directly as `tree` arguments, avoiding throwaway named definitions:

```f-tree
root main sequence {
    retryer(
        lambda sequence {
            fetch_data()
            validate_data()
        },
        notify_failure()
    )
}
```

---

## Parameter Scoping Rules

Forester does **not** perform closure-style parameter capturing. The semantics are:

| Argument type | Resolved when? |
|---|---|
| **Static constants** (strings, numbers, objects) | Captured at definition site and passed as-is |
| **Blackboard pointers** (`&key`) | Resolved at the moment of invocation |

This means Blackboard pointer arguments always reflect the **live state** of the Blackboard at invocation time, not at the point where the HOT definition was written.

---

## Common Reusable Patterns

```f-tree
// Retry-or-notify
fallback retry_or_notify(action: tree, notify: tree) {
    retry(3) action(..)
    notify(..)
}

// Timed action with fallback
fallback timed(action: tree, limit_ms: number, on_timeout: tree) {
    timeout(limit_ms) action(..)
    on_timeout(..)
}

// Logged execution
sequence logged(label: string, action: tree) {
    log(label)
    action(..)
}

impl log(message: string);
```
