# Engine

The runtime engine of the framework is `Forester`.
It encompasses several components:
- Blackboard
- ActionKeeper
- Runtime Env
- Tracer


## Example

```rust
 use std::path::PathBuf;
 use forester::tracer::Tracer;
 use forester::runtime::builder::ForesterBuilder;
 use forester::runtime::action::Action;
 use forester::runtime::action::builtin::data::StoreData;
 use forester_rs::runtime::action::Action;
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
     
     forester.run(); //  forester.run_until( Some(100));
 }

```

## Tick limitation

`Forester` allows limiting how many ticks will be done by running `run_with(Some(number))`

## Runtime environment
The framework uses `tokio` as a platform to orchestrate threads and parallelize the job.
By default, it creates its own tokio runtime env. 
Nevertheless, if there is existing env, it can be provided in the `ForesterBuilder` 