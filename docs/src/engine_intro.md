# Runtime engine

The runtime part executes the given tree.

There are 3 major components of the engine part
- Engine itself (Forester)
- Blackboard
- Actions(including ActionKeeper)

The runtime is predominantly synchronous with asynchronous environment for the async actions. 
The blackboard is in-memory for now.

## General api

The entry point is a `ForesterBuilder` that allows to build `Forester` in a safe way.
Also, it is highly customizable.

```rust
 use std::path::PathBuf;
 use forester::tracer::Tracer;
 use forester::runtime::builder::ForesterBuilder;
 use forester::runtime::action::Action;
 use forester::runtime::action::builtin::data::StoreData;
 use forester_rs::runtime::action::builtin::data::StoreData;
 use forester_rs::runtime::builder::ForesterBuilder;
 use forester_rs::tracer::Tracer;
 
fn test(root:PathBuf){
     let mut root = PathBuf::new();

     let mut fb = ForesterBuilder::new();
     fb.main_file("main.tree".to_string());
     fb.root(root);
     fb.register_action("store", Action::sync(StoreData));
     
     fb.tracer(Tracer::default());
     fb.bb_load("db/db.json".to_string());
     let forester = fb.build().unwrap();
     
     let r = forester.run().unwrap();
 }


```