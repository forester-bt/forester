# Fallback

A Fallback ticks children sequentially until someone returns a `Success`.
Otherwise, if all children return `Failure`, the node returns `Failure`.

In the language, the tree definitions and lambda invocations of this element are marked with the key word `fallback`.

```f-tree
cond is_busy()
impl take_from_others()

root main  {
    fallback {
        any_tasks() // goes farther if the first actions is failure
        do_it()
    }
}
```

## Common behavior
- When it gets the first `tick` it switches to state `running`
- When a child returns `success` it stops the execution and returns `success`
- If a child returns `running`, the node returns `running` as well
- If a child returns `failure`, the node proceeds to the next child
    - if this is a final child, it returns `failure`
- When a node is restarted or halted the process starts from the beginning

## Intention
Often, it is used for making conditions.
The script below emulates a simple condition that needs to do before
```f-tree
cond can_take(sub:object)
impl move_to(sub:object)
impl take(sub:object)

root main sequence {
    fallback {
        can_take(item)
        move_to(item)
    }
    take(item)

}
```
using a programming language, it could be the following:
```rust
fn main(item:String){
    if !can_take(item) {
        move_to(item)
    }
    take(item)
}
```

# Subtypes

There is one subtype that brings a few subtleties to the common process


## Reactive Fallback

This Fallback defines in the language with the keyword `r_fallback` and has the following peculiarity:
The fallback restarts all children on the next tick if someone returned `running`:

```f-tree
...
root main {
    r_fallback {
        needs_to_charge()    // returns failure
        action()  // returns running
        fin_and_save()
    }
}
```

The node `action` returns `running` and the whole sequence returns `running`
but on the next tick it starts from the node `needs_to_charge` again.

`r_fallback` will halt the `running` child to allow a graceful shutdown if a prior child changes from `failure` to `success`. In the above example, if `needs_to_change` returned `success` on the second tick then `action` would be halted before `r_fallback` returned `success` itself.

Halting must be performed as quickly as possible. Note that currently only build-in flow, built-in decorator and sync action nodes are halted, async and remote actions are not.
