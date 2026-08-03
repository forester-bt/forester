# Lambdas (Anonymous Inline Sub-Trees)

A **Lambda** is an anonymous, inline sub-tree that is defined and executed at the point of invocation — no named definition required.

Lambdas are ideal for one-off logic that is too simple to justify a named definition, or for passing inline behavior directly as a Higher-Order Tree argument.

---

## Key Properties

- **No name**: Lambdas are anonymous — they cannot be referenced from elsewhere.
- **No parameters**: Lambdas do not accept arguments. They operate on Blackboard state directly.
- **Flow nodes only**: Lambdas can only contain flow control nodes (`sequence`, `fallback`, `parallel`, decorators) and action invocations. Action signatures must still be declared with `impl` elsewhere.
- **Unique instances**: Each lambda definition creates a distinct node in the compiled tree.

---

## Basic Syntax

Any inline `sequence`, `fallback`, or `parallel` block without a name is a lambda:

```f-tree
import "std::actions"

impl job();

root main sequence {
    // Lambda: unnamed inline sequence
    sequence {
        job()
        job()
        job()
    }
    
    // Lambda: unnamed fallback with nested lambdas
    fallback {
        sequence {
            job()
            job()
        }
        // Decorator on a single-child lambda (brackets omitted)
        retry(3) job()
    }
}
```

---

## Lambdas as Higher-Order Tree Arguments

Lambdas can be passed directly as `tree`-typed arguments to Higher-Order Tree definitions. This avoids the need to create throwaway named definitions just to pass a block of logic:

```f-tree
impl savepoint();
impl fetch_data();
impl validate();
impl store_result();

sequence bookmarked(action: tree) {
    savepoint()
    action(..)
    savepoint()
}

root main sequence {
    // Pass an inline lambda as the 'action' argument
    bookmarked(
        sequence {
            fetch_data()
            validate()
            store_result()
        }
    )
    
    // Named argument syntax
    bookmarked(
        action = fallback {
            fetch_data()
            store_result()
        }
    )
}
```

---

## When to Use a Lambda vs. a Named Definition

| Situation | Recommendation |
|---|---|
| One-off logic used in a single place | **Lambda** |
| Logic reused in 2+ places | **Named sub-tree definition** |
| Passed as a HOT argument inline | **Lambda** |
| Needs its own parameters | **Named sub-tree definition** |