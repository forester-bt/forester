# Parallel 
A parallel node provides so-called pseudo-parallelism. 
It ticks all children sequentially but in one tick. 
Therefore, regardless of the result that current child returns the node proceeds to the next one.
The node returns `success` if all children return `success` and `failure` if at least one child returns `failure` 
and `running` otherwise.

In the language, it is defined with the keyword `parallel` and has the following syntax:
```f-tree
impl store(key:string, value:string); // store a string value to a key in blackboard 

root main {
    parallel {
        store("a","1") // first tick ticks but waits the result 
        store("b","2") // this node will be ticked in the same tick
    }
}
```

## Common behaviour
In general, it has resemblance to `sequence` node but with a few differences:

- When it gets the first `tick` it switches to state `running`
- When a child returns `success` it proceeds to the next one and ticks it
    - if this is a final child, it returns `success`
- If a child returns `running`, the node proceeds to the next one and ticks it
    - after that the node returns `running` as well
- If a child returns `failure`, the node proceeds to the next one and ticks it
    - after that the node returns `failure` as well
- When a node is restarted, the process starts from the beginning

## Intention
Often, it is used to run two independent (often async) actions
```f-tree
root main sequence {
        clean_current_room() // async impl that immidiately returns running  
        prepare_next_room()  // can be sync impl that returns success or failure
}
```

## Peculiarities

Since the actions are kicked off in the pseudo-parallel manner, 
it needs to be aware of the following peculiarities:

- The order of children is not important. All children will be ticked in the same tick.
- The node does wait for the result of all children.
- If a child returns `running` the node will return `running` as well.
- If a child returns `failure` or `success` but another child returns `running` the node will return `running` as well.
  - The next tick the finished nodes will be skipped and the node will tick the running node.
