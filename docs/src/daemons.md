# Daemons (Background Processes)

**Daemons** are long-running background tasks that execute concurrently alongside the behavior tree. They share the same Tokio runtime environment as the engine and have direct access to the Blackboard.

Common uses:
- **Sensor polling**: Continuously reading hardware or network sensor streams and writing values to the Blackboard.
- **Message publishing**: Sending telemetry or heartbeat events to external systems.
- **AI context maintenance**: Streaming LLM token output or monitoring API rate limits in the background.
- **Watchdogs**: Monitoring tree health or enforcing resource constraints.

> **Performance note**: Daemons run in the same async runtime as the tree. Heavy daemon workloads can directly impact tick loop performance. Keep daemon logic lightweight, or offload expensive work to separate processes.

---

## Daemon Types

| Type | Trait | Stop Mechanism |
|---|---|---|
| **Sync** | `DaemonFn` | `StopFlag` — an `AtomicBool` that flips to `true` when the engine requests shutdown |
| **Async** | `AsyncDaemonFn` | `CancellationToken` — a Tokio one-shot cancellation channel |

---

## Implementing Daemons

### Sync Daemon

Poll the `StopFlag` in a loop. When it becomes `true`, the daemon should exit promptly:

```rust
use forester_rs::runtime::env::daemon::{DaemonFn, DaemonContext, StopFlag};
use std::sync::atomic::Ordering::Relaxed;

struct SensorPollerDaemon;

impl DaemonFn for SensorPollerDaemon {
    fn perform(&mut self, ctx: DaemonContext, signal: StopFlag) {
        while !signal.load(Relaxed) {
            std::thread::sleep(std::time::Duration::from_millis(50));
            
            let mut bb = ctx.bb.lock().unwrap();
            let reading = read_sensor(); // your sensor read logic
            bb.put("sensor_value".to_string(), RtValue::int(reading)).unwrap();
        }
    }
}
```

---

### Async Daemon

Use `tokio::select!` to respond to cancellation alongside your periodic work:

```rust
use forester_rs::runtime::env::daemon::{AsyncDaemonFn, DaemonContext};
use tokio_util::sync::CancellationToken;
use std::pin::Pin;
use std::future::Future;

struct AsyncSensorPollerDaemon;

impl AsyncDaemonFn for AsyncSensorPollerDaemon {
    fn prepare(&mut self, ctx: DaemonContext, signal: CancellationToken) -> Pin<Box<dyn Future<Output = ()> + Send>> {
        Box::pin(async move {
            loop {
                tokio::select! {
                    _ = signal.cancelled() => {
                        // Gracefully shut down
                        return;
                    }
                    _ = tokio::time::sleep(std::time::Duration::from_millis(10)) => {
                        let mut bb = ctx.bb.lock().unwrap();
                        let reading = fetch_remote_sensor().await;
                        bb.put("sensor_value".to_string(), RtValue::int(reading)).unwrap();
                    }
                }
            }
        })
    }
}
```

---

## Registering Daemons

### At Startup via `ForesterBuilder`

Register daemons before building the engine. Named daemons can be controlled from within the tree using built-in actions:

```rust
use forester_rs::runtime::env::daemon::Daemon;

// Named daemon (controllable from the tree via stop_daemon / daemon_alive)
fb.register_named_daemon("sensor_poller".to_string(), Daemon::sync(SensorPollerDaemon));

// Anonymous daemon (runs for the lifetime of the engine, no tree control)
fb.register_daemon(Daemon::a_sync(AsyncSensorPollerDaemon));
```

### At Runtime from Inside an Action

Daemons can also be started dynamically during tree execution from within a sync action:

```rust
use forester_rs::runtime::action::{Impl, RtArgs, Tick, TickResult};
use forester_rs::runtime::context::TreeContextRef;
use forester_rs::runtime::env::daemon::Daemon;

impl Impl for StartPollerAction {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let env = ctx.env().lock()?;
        env.start_daemon(Daemon::a_sync(AsyncSensorPollerDaemon), ctx.into());
        Ok(TickResult::success())
    }
}
```

---

## Controlling Daemons from the Tree

Two built-in standard library actions are available to control named daemons from `.tree` files:

```f-tree
import "std::actions"

root main sequence {
    // Start the main task
    execute_mission()
    
    // Check if background poller is still running
    daemon_alive("sensor_poller")
    
    // Stop the background poller when done
    stop_daemon("sensor_poller")
}
```

| Action | Description |
|---|---|
| **`daemon_alive(name)`** | Returns `Success` if the named daemon is still running, `Failure` otherwise. |
| **`stop_daemon(name)`** | Sends the stop signal to the named daemon and returns `Success`. |