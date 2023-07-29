# Setup

There are two ways to interact with Forester

## Using console utility for simulation and visualization

The console utility `f-tree` can be installed using `cargo`

```shell
cargo install f-tree
```

and then be used with

```shell
~ f-tree
A console utility to interact with Forester

Usage: f-tree <COMMAND>

Commands:
  sim   Runs simulation. Expects a simulation profile
  vis   Runs visualization. Output is in svg format.
  help  Print this message or the help of the given subcommand(s)

Options:
  -h, --help     Print help
  -V, --version  Print version
```


## As a dependency to run from a rust code

```toml
forester-rs = "*"
```


### From file system

```rust

 use std::path::PathBuf;
 use forester_rs::flow;
 use forester_rs::tracer::Tracer;
 use forester_rs::runtime::builder::ForesterBuilder;
 use forester_rs::runtime::action::Action;
 use forester_rs::runtime::action::builtin::data::StoreData;

fn main() {
    let mut fb = ForesterBuilder::from_file_system();
    fb.main_file("main.tree".to_string());
    fb.root(root);
    fb.register_action("store", Action::sync(StoreData));
    fb.tracer(Tracer::default());
    fb.bb_load("db/db.json".to_string());
    
    let forester = fb.build().unwrap();

    let result = forester.run().unwrap();
    println!("result {:?}",result);
}


```

### On the fly for small scripts

```rust

use std::path::PathBuf;
use forester_rs::flow;
use forester_rs::tracer::Tracer;
use forester_rs::runtime::builder::ForesterBuilder;
use forester_rs::runtime::action::Action;
use forester_rs::runtime::action::builtin::data::StoreData;

fn main() {
    let mut fb = ForesterBuilder::from_text();
    fb.register_action("cv",Action::sync(ReturnResult::success()));
    
    fb.text(r#"
        root main sequence {
            cv()
            cv()
            cv()
        }
    "#.to_string());
    let mut forester = fb.build().unwrap();

    let result = forester.run().unwrap();
    println!("result {:?}",result);
}


```


### Manually construct the trees

```rust

use std::path::PathBuf;
use forester_rs::flow;
use forester_rs::tracer::Tracer;
use forester_rs::runtime::builder::ForesterBuilder;
use forester_rs::runtime::action::Action;
use forester_rs::runtime::action::builtin::data::StoreData;

fn main() {
    let mut fb = ForesterBuilder::from_code();
    fb.register_action("cv",Action::sync(ReturnResult::success()));
    fb.add_rt_node(
          flow!(fallback node_name!(), args!();
              action!(),
              action!(),
              action!(),
              action!()
          )
    );
    let mut forester = fb.build().unwrap();

    let result = forester.run().unwrap();
    println!("result {:?}",result);
}


```