# Syntax

The syntax of the `tree` language is similar to any average scripting language namely it consists of:
- tree definitions: subtree that defines a prat of the complete tree.
- tree invocations: the calls of the tree definitions.
- imports: the instructions that enable import from the other files.
- parameters and arguments: that allow to pass the values and the other tree to the tree definitions.
- lambda:  the ability to define the anonymous tree definitions and invoke it at the same time.
- comments: the extra information.

Below, a simple example that shows aforementioned points

```f-tree
import "nested/impls.tree"
import "nested/impls.tree" {
    grasp => grasp_ball,
}

root place_ball_to_target fallback {
    place_to(
        obj = {"x":1 },
        operation = place([10]),
    )
    retry(5) ask_for_help()
}

sequence place_to(what:object, operation:tree){
    fallback {
        is_approachable(what)
        do_job(approach(what))
    }
    fallback {
         is_graspable(what)
         do_job(approach(what))
    }
    sequence {
         savepoint()
         operation(..)
    }
}

sequence place(where:array){
    is_valid_place(where)
    do_job(slowly_drop({"cord":1}))
}

sequence do_job(action:tree){
    savepoint()
    info_wrapper(action(..))
    savepoint()
}

sequence info_wrapper(action:tree){
    log("before action")
    action(..)
    log("before action")
}

impl log(text:string);

```
 
