# Daemons
Forester provides the conception of the background processes called daemons. \
Daemons are used to perform some actions in the background. \
For example, it can be used to publish some messages  or subscribe to the topics from the external system \
or to perform some actions on the blackboard in the background.

*The daemons are executed at the same runtime environment 
as a tree thus the daemons can affect the performance of the tree directly.*

## Daemon definition
The enum `Daemon` encapsulates a daemon function and provides the following variants:
- sync - the daemon function is synchronous and will be wrapped into async function.
- async - the daemon function is asynchronous and will be executed as is.


## How to stop the daemon
Since, the daemon is supposed to be a long-living background process, there is no way to predict when it will be stopped. \
Therefore, depending on the daemon type, the engine provides the following ways to stop the daemon:

### Sync daemon
The sync daemon function accepts the `StopSignal` as an argument. \
The `StopSignal` is a simple atomic boolean that initially false and when it switches to true, the daemon should be stopped. 

### Async daemon
The async daemon function accepts the `CancellationToken` as an argument. \
The `CancellationToken` is a mechanism from tokio that allows to stop the async function.(one shot channel)


## Examples of the daemon

```rust
struct DaemonSync;

impl DaemonFn for DaemonSync {
    fn perform(&mut self, ctx: DaemonContext, signal: StopFlag) {
        while !signal.load(Relaxed) {
            std::thread::sleep(std::time::Duration::from_millis(50));
            let mut bb = ctx.bb.lock().unwrap();
            let v = bb.get("test".to_string()).expect("no errors")
                .cloned().unwrap_or(RtValue::int(0));

            bb.put("test_daemon".to_string(), v).unwrap();
        }
    }
}

impl AsyncDaemonFn for DaemonSync {
    fn prepare(&mut self, ctx: DaemonContext, signal: CancellationToken) -> Pin<Box<dyn Future<Output=()> + Send>> {
        Box::pin(async move {
            loop {
                tokio::select! {
                _ = signal.cancelled() => {
                    return;
                }
                _ = tokio::time::sleep(std::time::Duration::from_millis(10)) => {
                    let mut bb = ctx.bb.lock().unwrap();
                    let v = bb.get("test".to_string()).expect("no errors")
                        .cloned().unwrap_or(RtValue::int(0));

                    bb.put("test_daemon".to_string(), v).unwrap();
                }
            }
            }
        })
    }
}
```

## Daemon registration

The daemon can be registered as follows:

### Using the tree builder

```rust

fn register(fb:ForesterBuilder){
    let signal = Arc::new(AtomicBool::new(false));
    fb.register_named_daemon("daemon".to_string(), Daemon::sync(DaemonSync));
    fb.register_daemon(DaemonSync(signal));
}


```

### Using the runtime environment

```rust
impl Impl for Action {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let env = ctx.env().lock()?;
        env.start_daemon(Daemon::a_sync(DaemonSync), ctx.into());
        Ok(TickResult::success())
    }
}
```

## BuiltIn actions
There are 2 built-in actions that can be used to control the daemons:
 - `stop_daemon` - stops the daemon by the name
 - `daemon_alive` - check if the daemon is alive by the name