# Actions

Actions are the leaves of the tree. They require some implementation to be executed.


## Action types

There are three types of actions available at that moment:
- Sync actions: the actions that block the flow until the action get done.
- Async action: initiate the calculation at the different thread and return `running` immediately.
- Remote action: send the blocking request (http) to the remote host.

- For heavy actions, preferably to use `async actions`.

## Traits

The action trait implements two functions, `tick()` and `halt()`.

The `tick()` function is the main entry point of the action and will be called whenever the node is executed.

The `halt()` function is used to notify a `running` action that a reactive flow node (e.g. `r_sequnce`) has changed the control flow. This means the previously `running` action won't be called again, or won't be called for a while, and so should gracefully clean up. The `halt()` function has a default no-op implementation that can be used if no clean up is necessary.

Actions must halt as quickly as possible, and should not block the execution.

### `Impl` for sync actions

Sync actions are the only actions that currently implement the `halt()` function.

```rust
pub trait Impl {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick;

    fn halt(&self, args: RtArgs, ctx: TreeContextRef) -> RtOk {
        // Default halt is a no-op function.
        let _ = args;
        let _ = ctx;
        Ok(())
    }
}

```

### `ImplAsync` for async actions
```rust
pub trait ImplAsync: Sync + Send {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick;
}
```

Where `args` are the given arguments from the tree definition and invocation and `ctx`
is a reference of the invocation context with `bb` and `tracer`.

## Mutability
The actions are intentionally stateless thus they can't mutate.
Therefore, it is better off to use blackboard to keep some data between the calls.

## How to register action

```rust
fn simple_delay() {
    let mut forester_builder = fb("decorators/simple_delay");

    forester_builder.register_sync_action("store", StoreData);

}
```

## Async actions

The async actions are executed in the multithreading environment and return the `running` tick result instantly.
It does not block the execution of the tree and can be used in parallel nodes, etc.

On the other hand, every time when the tree is reloaded, the tick number is increased that can exceed the limit on ticks
if the system has it. Therefore, it needs to take into account (when forester runs with the limit of ticks.)


## Remote actions

The remote actions are the actions that send the request to the remote host and wait for the response.
For now, it is only http requests with json body and json response.

The remote actions can have access to the blackboard and the tracer if the http-server is running (see [http-server](./engine.md#http-server)).

The remote actions should implement `ImplRemote` trait:

```rust

pub trait ImplRemote: Sync + Send {
    fn tick(&self, args: RtArgs, ctx: TreeRemoteContextRef) -> Tick;
}
```

Where `args` are the given arguments from the tree definition and invocation and `ctx` has the information about the http_server:
```rust
pub struct TreeRemoteContextRef<'a> {
    pub curr_ts: Timestamp, // current timestamp
    pub port: u16,          // port of the http server, to access the blackboard and tracer
    pub env: &'a mut RtEnv, // runtime env to execute the http request
}
```

The default implementation of the `tick` method is available in `forester_rs::runtime::action::builtin::remote::RemoteHttpAction`:

```rust
pub struct RemoteHttpAction {
    url: String,
    serv_ip: Option<String>,
}
```
it accepts the url and the ip of the http server (if it is not localhost, which is a default parameter).

The message is the following:
```rust
pub struct RemoteActionRequest {
    pub tick: usize,            // current tick
    pub args: Vec<RtArgument>,  // arguments from the tree
    pub serv_url: String,       // url of the http server to get access to the blackboard and tracer
}
```

The response is the following a `TickResult`.

How to implement the client side, please see [remote action lib](./rem_action.md).


## Default actions

By default, there are several implementations for http and interactions with bb are available in

```rust
use forester_rs::runtime::action::builtin::*;
```
