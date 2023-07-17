# Actions

Actions are the leaves of the tree. They require some implementation to be executed.


## Action types

There are 2 types of actions available at that moment:
- Sync actions: the actions that block the flow until the action get done.
- Async action: initiate the calculation at the different thread and return `running` immediately.

- For heavy actions, preferably to use `async actions`.

## Traits

### `Impl` for sync actions

```rust
pub trait Impl {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick;
}
```

#### `ImplAsync` for async actions
```rust
pub trait ImplAsync: Sync + Send {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick;
}
```

Where `args` are the given arguments from the tree definition and invocation and `ctx` 
is a reference of the invocation context with `bb` and `tracer`  

## Mutability
The actions are intentionally stateless thus they can't mutate.
Therefore, it is better off to use blackboard to keep some data between the calls.

## How to register action

```rust
fn simple_delay() {
    let mut forester_builder = fb("decorators/simple_delay");

    forester_builder.register_action("store", Action::sync(StoreData));
 
}
```

## Async actions

The async actions are executed in the multithreading environment and return the `running` tick result instantly.
It does not block the execution of the tree and can be used in parallel nodes, etc.

On the other hand, every time when the tree is reloaded, the tick number is increased that can exceed the limit on ticks 
if the system has it. Therefore, it needs to take into account (when forester runs with the limit of ticks.)


## Default actions

By default, there are several implementations for http and interactions with bb are available in  

```rust
use forester_rs::runtime::action::builtin::*;
```
