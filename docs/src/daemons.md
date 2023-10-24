# Daemons
Forester provides the conception of the background processes called daemons. \
Daemons are used to perform some actions in the background. \
For example, it can be used to publish some messages  or subscribe to the topics from the external system \
or to perform some actions on the blackboard in the background.

*The daemons are executed at the same runtime environment 
as a tree thus the daemons can affect the performance of the tree directly.*

## Daemon definition

There is a specific trait `Daemon` that should be implemented for the daemon.
```rust
pub trait Daemon: Send + Sync {
    fn perform(&mut self, ctx: DaemonContext);
    fn signal(&self) -> StopSignal;
}
```

The perform method is called by the engine in the background in the different thread (asynchronously). \
The daemon should be able to stop the execution as soon as it receives the stop signal. \
The stop signal is returned by the signal method. \

therefore most likely if the daemon should carry the state it should be wrapped into the `StopSignal = Arc<Mutex<bool>>`

The signal can be in the following cases:
 - when the tree is stopped
 - by the built-in action `stop_daemon` (see below)
 - another action using, `rt_env.stop_daemon()`

## Example of the daemon

```rust
struct DaemonSync(StopSignal);

impl Daemon for DaemonSync {
    fn perform(&mut self, ctx: DaemonContext) {
        while !self.signal().load(Relaxed) {
            std::thread::sleep(std::time::Duration::from_millis(50));
            let mut bb = ctx.bb.lock().unwrap();
            let v = 
                bb.get("test".to_string())
                    .expect("no errors")
                    .cloned()
                    .unwrap_or(RtValue::int(0));

            bb.put("test_daemon".to_string(), v).unwrap();
        }
    }

    fn signal(&self) -> StopSignal {
        self.0.clone()
    }
}
```

## Daemon registration

The daemon can be registered as follows:

### Using the tree builder

```rust

fn register(fb:ForesterBuilder){
    let signal = Arc::new(AtomicBool::new(false));
    fb.register_named_daemon("daemon".to_string(), DaemonSync(signal));
    fb.register_daemon(DaemonSync(signal));
}


```

### Using the runtime environment

```rust
impl Impl for Action {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let env = ctx.env().lock()?;
        env.start_daemon(Box::new(Daemon::new()), ctx.into());
        Ok(TickResult::success())
    }
}
```

## BuiltIn actions
There are 2 built-in actions that can be used to control the daemons:
 - `stop_daemon` - stops the daemon by the name
 - `daemon_alive` - check if the daemon is alive by the name