# Installation

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

## As a dependency to run from a rust code and

```toml
forester-rs = "0.1.0"
```

```rust

use forester_rs::runtime::action::builtin::ReturnResult;
use forester_rs::runtime::action::Action;
use forester_rs::runtime::action::Tick;
use forester_rs::runtime::builder::ForesterBuilder;
use forester_rs::runtime::RtResult;

fn main() {
    let mut fb = ForesterBuilder::new();
    fb.root(PathBuf::from("folder"));
    fb.register_action("cv",Action::sync(ReturnResult::success()));

    let mut forester = fb.build().expect("the params are good");

    let result = forester.run().unwrap();
    println!("result {:?}",result);
}


```