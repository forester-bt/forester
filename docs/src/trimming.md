# Trimming

The engine provides a simple way to change the runtime tree or other components on a fly during the execution.

## The premises of trimming

Why do we need to have this process?

### Performance/Optimization tasks

The execution can be optimized from the performance/memory point of view as JIT compilers act.
Thus, it enables the transformation of the execution folding/caching of some nodes.

### Logical tasks

When the execution needs to be changed according to some logical premises or incentives based on the runtime
information (like a process of reinforcement learning)

### Research tasks

The possibility to perform research on how the process can be changed in case to compare the results in the same
environment.

## The structure of Trimming

The trimming consists of several simple components:

### Trimming task
A task that will be executed. 
Typically, the task can decide whether it needs to be postponed, rejected or get to execute. 
The implementation touches the different components like trimming of a runtime tree.
```rust
pub enum TrimTask {
    RtTree(Box<dyn RtTreeTrimTask>),
}

impl TrimTask {
    /// the main method to execute
    pub fn process(&self, snapshot: TreeSnapshot<'_>) -> RtResult<TrimRequest> {..}
}
```

### TrimRequest
A request to trim. Since, there are no guarantees of the specific order of the different tasks or even the moment of time (in terms of ticks) 
when it will be executed (for instance, the nodes that this task tries to trim are running and therefore this task will be postponed), 
the request has influence on the possible execution of itself. 

The possible states:
- Reject: The task can reject itself, when it finds out that, for instance, another task performed the same changed 
or made the tree unsuitable for the current changes.
- Skip: Skip the current tick. When the conditions are inappropriate. 
For instance, the task waits for a specific data in bb or a particular tick or anything else.
- Attempt: Attempt to trim

```rust
#[derive(Debug)]
pub enum TrimRequest {
    Reject,
    Skip,
    Attempt(RequestBody),
}
```

### RequestBody
Just a structure that bears all changes of the request.

### Validations

Under the hood, the engine tries to validate a given request and ensure that the tree will not be corrupted.
For now, it performs only the check if the nodes of the tree that are about to be replaced are not running.

## Constrains

There is no way to foresee and guarantee the possible order or the possible moment when the trimming task will be executed, 
or even will it be executed at all, therefore, better to pursue to create the task idempotent and validate the incoming state diligently.

## Example

```rust
 use forester_rs::*;
 use forester_rs::runtime::forester::Forester;
 use forester_rs::runtime::rtree::builder::RtTreeBuilder;
 use forester_rs::runtime::RtResult;
 use forester_rs::runtime::trimmer::task::{RtTreeTrimTask, TrimTask};
 use forester_rs::runtime::trimmer::{RequestBody, TreeSnapshot, TrimRequest};
 use forester_rs::runtime::rtree::builder::RtNodeBuilder;
 use forester_rs::runtime::rtree::rnode::RNodeName;
 use forester_rs::runtime::args::RtArgs;

 fn smoke(mut forester: Forester) {

     forester.add_trim_task(TrimTask::rt_tree(Test));
     let result = forester.run_until(Some(100)).unwrap();
     println!("{}",result);
 }

 struct Test;

 // just take a not and manually replace it.
 impl RtTreeTrimTask for Test {
     fn process(&self, snapshot: TreeSnapshot<'_>) -> RtResult<TrimRequest> {
         if snapshot.tick < 90 {
             Ok(TrimRequest::Skip)
         } else {
             let tree = snapshot.tree;
             let id = tree
                 .nodes
                 .iter()
                 .find(|(_, n)| {
                     n.name()
                         .and_then(|n| n.name().ok())
                         .filter(|n| n.as_str() == "fail_empty")
                         .is_some()
                 })
                 .map(|(id, _)| id)
                 .unwrap();
             let mut rtb = RtTreeBuilder::new_from(tree.max_id() + 1);
             rtb.set_as_root(action!(node_name!("success")), id.clone());

             Ok(TrimRequest::attempt(RequestBody::new(
                 rtb,
                 Default::default(),
             )))
         }
     }
 }



```