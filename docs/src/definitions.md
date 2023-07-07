# Tree definition

The tree definition denotes a part of the tree (so-called subtree) that defines an independent description and can 
- have the input parameters and accept arguments including other trees definitions
- invoke other tree definitions and get invoked by others (except `root`)

There are the following types of the definitions:
- Flow:
  the [core part](https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control)#Control_flow_node) of the behavior tree framework.
The nodes define a logic of processing the tree itself, navigating for the next step.
- Lambda: The anonymous definition of subtree with instant invocation at this place.
- Decorator: the atomic built-in tree definition that has one child and can enrich or transform the child result according to its type.  
- Actions: the leaves of the tree bearing the business logic.