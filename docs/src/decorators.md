# Decorators

A **Decorator** is a specialized single-child node that wraps a child node and modifies its execution behavior or transforms its return state (`Success`, `Failure`, or `Running`).

Every decorator accepts **exactly one child node** (which can be a single action, a sub-tree, or a block wrapped in `{ }`).

---

## Built-In Decorator Reference

| Decorator | Parameters | Default | Description |
|---|---|---|---|
| **`inverter`** | None | — | Inverts child `Success` to `Failure`, and `Failure` to `Success`. (`Running` is untouched). |
| **`force_success`** | None | — | Always returns `Success` regardless of whether the child succeeds or fails. |
| **`force_fail`** | None | — | Always returns `Failure` regardless of child outcome. |
| **`repeat(N)`** | `count: number` | `0` (Infinite) | Repeats execution of the child $N$ times. If count is `0`, repeats infinitely. |
| **`retry(N)`** | `attempts: number` | `0` (Infinite) | Re-ticks the child up to $N$ times if it returns `Failure`. If attempts is `0`, retries indefinitely. |
| **`timeout(ms)`** | `limit: number` | `1000` | Limits child execution time in milliseconds. Halts child and returns `Failure` if exceeded. |
| **`delay(ms)`** | `wait: number` | `0` | Delays the initial execution of the child for the specified duration in milliseconds. |

---

## Placement of Decorators

Decorators in Forester can be placed in two main ways:

### 1. In Tree / Root Definitions (Header Decorators)
Decorators can be placed directly in the declaration header of a `root` node or sub-tree definition:

```f-tree
// Root node wrapped in a repeat decorator
root main_fixed repeat(5) {
    execute_cycle()
}

// Sub-tree definition wrapped with a retry decorator
sequence retryable_task retry(3) {
    perform_step()
}
```

### 2. At Invocation Sites
Decorators can be placed directly before an action invocation, sub-tree call, or inline lambda:

```f-tree
root main sequence {
    // Decorator on an action invocation
    retry(3) perform_task()
    
    // Decorator on a sequence block
    timeout(1000) sequence {
        fetch_data()
    }
    
    // Decorator on an inline lambda
    retry(5) lambda sequence {
        fetch_sensor_data()
        validate_reading()
    }
}
```

---

## Detailed Examples

### 1. Inverter (`inverter`)
Inverts the result of condition checks or actions:

```f-tree
import "std::actions"

root main sequence {
    // Succeeds if obstacle_detected() fails
    inverter obstacle_detected()
    move_forward()
}

impl obstacle_detected();
impl move_forward();
```

---

### 2. Result Overrides (`force_success` & `force_fail`)
Enforce specific outcome states regardless of child execution:

```f-tree
root main sequence {
    // Attempt cleanup; continue even if cleanup returns Failure
    force_success cleanup_temp_files()
    proceed_main_task()
}

impl cleanup_temp_files();
impl proceed_main_task();
```

---

### 3. Repeat (`repeat`)
Executes a child multiple times (or infinitely for continuous control loops):

```f-tree
// Infinite execution loop
root main_idle repeat {
    background_health_check()
}

// Fixed 5-cycle loop
root main_fixed repeat(5) {
    execute_cycle()
}

impl background_health_check();
impl execute_cycle();
```

---

### 4. Retry (`retry`)
Automatically retries failing actions (ideal for network calls or LLM tool invocations):

```f-tree
root main sequence {
    // Retry up to 5 times if call_llm returns Failure
    retry(5) call_llm({"prompt": "analyze_image"})
}

impl call_llm(params: object);
```

---

### 5. Timeout (`timeout`) & Delay (`delay`)
Controls timing for async actions:

```f-tree
root main sequence {
    // Wait 500ms before starting initial tick
    delay(500) initialize_sensors()
    
    // Shut down async action if it runs longer than 3000ms
    timeout(3000) fetch_remote_data()
}

impl initialize_sensors();
impl fetch_remote_data();
```