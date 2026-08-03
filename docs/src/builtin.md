# Built-In Standard Library Actions (`std::actions`)

Forester provides a built-in standard library of common utility actions, Blackboard operations, and state helpers.

To use standard library actions, import `std::actions` at the top of your `.tree` file:

```f-tree
import "std::actions"

root main sequence {
    store("session_id", "12345")
    equal("session_id", "12345")
}
```

Selective imports with aliasing can also be used:

```f-tree
import "std::actions" {
    store => set_blackboard_key,
    fail => throw_failure,
}
```

---

## Standard Action Reference

### 1. Terminal / Flow Control Actions

| Action | Arguments | Return State | Description |
|---|---|---|---|
| **`success()`** | None | `Success` | Instantly returns `Success`. Useful as stub or default branch. |
| **`fail(reason)`** | `reason: string` | `Failure` | Instantly fails execution with an explicit error reason. |
| **`fail_empty()`** | None | `Failure` | Instantly fails execution without a reason message. |
| **`running()`** | None | `Running` | Returns `Running`. Keeps node active on subsequent ticks. |
| **`sleep(duration)`**| `duration: number` | `Success` | Non-blocking sleep for $N$ milliseconds before returning `Success`. |

---

### 2. Blackboard Memory Actions

| Action | Arguments | Description |
|---|---|---|
| **`store(key, value)`** | `key: string, value: string` | Stores a value into the Blackboard under `key`. |
| **`equal(key, expected)`**| `key: string, expected: string` | Compares Blackboard value under `key` to `expected`. Returns `Success` if equal, `Failure` otherwise. |
| **`store_tick(key)`** | `key: string` | Stores the current engine tick number into `key`. |
| **`lock(key)`** | `key: string` | Locks a Blackboard key to prevent modifications by other nodes. |
| **`unlock(key)`** | `key: string` | Unlocks a previously locked Blackboard key. |

---

### 3. I/O & Network Actions

| Action | Arguments | Description |
|---|---|---|
| **`http_get(url, bb_key)`** | `url: string, bb_key: string` | Performs an HTTP GET request to `url` and stores the response payload into `bb_key`. |

---

## Code Example

```f-tree
import "std::actions"

root main sequence {
    // 1. Initialize Blackboard state
    store("agent_status", "initializing")
    
    // 2. Perform work
    fallback {
        http_get("https://api.example.com/health", "health_response")
        fail("Health check API unreachable")
    }
    
    // 3. Verify status
    equal("agent_status", "initializing")
    
    // 4. Update status to active
    store("agent_status", "active")
}
```