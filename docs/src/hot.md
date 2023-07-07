# Higher order tree

The definitions can be passed as arguments in invocations for other definitions.
The definitions should accept `tree` as a parameter.

To invoke the definition, coming from parameters, the definition should have a name and '(..)' after,
like that:`operation(..)`

## Motivation

To reduce the amount of redundancy in implementing some logic.
The higher order tree enables to construct abstractions that will be easily 
used in the next tree definitions reducing the amount of code. 

## Syntax

```f-tree
...

// the checked_task declares acceptance of 2 tree definitions
fallback checked_task(cond:tree, task:tree){
    // invoke a tree definition from parameters
    cond(..)
    // invoke a tree definition from parameters
    task(..)
}

sequence handle(item:object) {
    // higher order invocation in arguments
    checked_task(close_enough(item), approach(item))
    checked_task(is_graspable(item), grasp(item))
    // The lambdas can be used as higher-order tree as well
    checked_task(enough_space(item), sequence {
        move(item)
        save(tem)
    })
}
```

## Parameters
For now, the tree does not perform the parameter capturing.
It means the following:
- static constants are passed as is
- pointers are resolved at the moment of invocation

