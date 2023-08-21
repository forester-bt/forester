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
Nevertheless, if there is existing env, it can be provided in `ForesterBuilder` 

## Http Server

The server provides an option to set up the http server to interact with the tree. 
The server exposes the access to the blackboard and the tracer.

To turn on the server, it is required to provide the port number in the `ForesterBuilder`.
*When the forester finishes the execution of the tree, the server will be shut down.*

```rust
 fn serv(fb:ForesterBuilder){
     fb.http_serv(10000); // the port then will be sent to the remote actions as well
 }
```

## The API

The server exposes the following endpoints:

- `GET /tracer/print` - print the tracer
- `POST /tracer/custom` - add a custom message to the tracer. It accepts the json body with `CustomEvent`
- `GET /bb/:key/lock` - lock the key
- `GET /bb/:key/unlock` - unlock the key
- `GET /bb/:key/locked` - check if the key is locked
- `GET /bb/:key/contains` - check if the key is in the bb
- `GET /bb/:key/take` - take the key from the bb
- `POST /bb/:key` - put the key to the bb. It accepts the json body from `RtValue`
- `GET /bb/:key` - get the key from the bb
- `GET /` - health check. Returns 'Ok'