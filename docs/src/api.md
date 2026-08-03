# Setup & Quick Start

Forester can be used as a CLI tool for simulation/analysis, embedded as a Rust dependency, or integrated remotely from Python.

---

## 1. CLI Tool Setup (`f-tree`)

Install the `f-tree` command-line utility via `cargo`:

```shell
cargo install f-tree
```

Verify installation and view available subcommands:

```shell
f-tree --help
```

### Common CLI Subcommands

* **Simulation (`f-tree sim`)**: Run behavior tree execution against simulated action stubs.
* **Visualization (`f-tree vis`)**: Generate SVG/Graphviz visual diagrams of `.tree` files.
* **Tree Validation (`f-tree check`)**: Statically validate tree syntax, types, and imports.

---

## 2. Rust Project Integration

Add `forester-rs` to your `Cargo.toml`:

```toml
[dependencies]
forester-rs = "0.2"
```

### Example 1: Loading Trees from the Filesystem

```rust
use forester_rs::runtime::builder::ForesterBuilder;
use forester_rs::runtime::action::Action;
use forester_rs::tracer::Tracer;

fn main() {
    let mut fb = ForesterBuilder::from_file_system();
    fb.main_file("main.tree".to_string());
    fb.root("main");
    fb.tracer(Tracer::default());
    fb.bb_load("db/initial_state.json".to_string());
    
    let mut forester = fb.build().expect("Failed to build Forester runtime");
    let result = forester.run().expect("Tree execution failed");
    
    println!("Execution completed with result: {:?}", result);
}
```

### Example 2: In-Memory / Inline DSL Script Execution

```rust
use forester_rs::runtime::builder::ForesterBuilder;

fn main() {
    let mut fb = ForesterBuilder::from_text();
    
    fb.text(r#"
        import "std::actions"
        
        root main sequence {
            action_a()
            action_b()
        }
        
        impl action_a();
        impl action_b();
    "#.to_string());

    let mut forester = fb.build().expect("Failed to parse inline script");
    let result = forester.run().expect("Execution failed");
    
    println!("Result: {:?}", result);
}
```

### Example 3: Programmatic Tree Construction

```rust
use forester_rs::runtime::builder::ForesterBuilder;
use forester_rs::flow;

fn main() {
    let mut fb = ForesterBuilder::from_code();
    
    // Programmatically construct tree nodes using Rust macros
    fb.add_rt_node(
        flow!(fallback "recovery_root", args!();
            action!("check_sensor"),
            action!("reset_state")
        )
    );
    
    let mut forester = fb.build().unwrap();
    let result = forester.run().unwrap();
    
    println!("Result: {:?}", result);
}
```

---

## 3. Python Integration (Remote Actions for AI Tools)

For Python-based AI workflows (LangChain, LlamaIndex, custom LLM tools), install the Python remote-action client:

```shell
pip install forester-http-ra-py
```

The Python client connects your Python tools over HTTP to the Forester Rust runtime, executing remote actions seamlessly.