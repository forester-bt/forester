# Simulation

Forester provides a simulation environment to execute a behavior tree by replacing actual action implementations with stubs. This allows you to validate tree logic and test execution branches under specific conditions without writing the application code.

Using a simulation profile, you can inject a specific Blackboard state, trace execution changes, and generate visual graphs of the tree.

## Preparations

### Configuration Profile

**Note: All paths in the configuration file can be either absolute or relative to the root folder.**

The YAML configuration file contains simulation settings and stub definitions for actions.

Example configuration:

```yaml
config:
  tracer: 
    file: gen/main.trace
    dt_fmt: "%d %H:%M:%S%.3f"
  graph: gen/main.svg
  bb:
    dump: gen/bb.json
  max_ticks: 10

actions:
  -
    name: task
    stub: failure
    params:
      delay: 100
```

#### `config` Section

| Setting | Description | Default | Example |
| :--- | :--- | :--- | :--- |
| `tracer.file` | File path to write the execution trace. | None (disabled) | `gen/main.trace` |
| `tracer.dt_fmt` | Datetime format for the trace logs. | None | `"%d %H:%M:%S%.3f"` |
| `graph` | File path to output the SVG tree visualization. | None (disabled) | `gen/main.svg` |
| `bb.dump` | File path to dump the final Blackboard state as JSON. | None (disabled) | `gen/bb.json` |
| `bb.load` | File path of a JSON file used to initialize the Blackboard before execution. | None (disabled) | `gen/init_bb.json` |
| `max_ticks` | Maximum number of ticks before the simulation forcefully terminates. | `0` (unlimited) | `10` |
| `http.port` | Port for the HTTP server to listen for remote action callbacks. | None (disabled) | `8080` |

#### `actions` Section

The `actions` section is an array mapping action names to their stubbed behavior.

| Setting | Description | Default | Example |
| :--- | :--- | :--- | :--- |
| `name` | Name of the target action. | **Required** | `task` |
| `stub` | Stub behavior (`success`, `failure`, `random`, `remote`). | **Required** | `success` |
| `params.delay` | Execution delay in milliseconds. | `0` | `100` |
| `params.url` | *(Remote stub)* URL of the remote server. | **Required** for `remote` | `http://localhost:10000/action` |
| `params.server` | *(Remote stub)* Callback URL the remote action uses to access the Blackboard. | `http://localhost` | `http://localhost:8080` |


### Default Profile
A simulation can run without a specific profile. In this case, Forester replaces all unimplemented actions with a `success` stub. No traces, graphs, or dumps are generated.

### Stubs

- `success`: Always returns `Success`.
- `failure`: Always returns `Failure`.
- `random`: Returns `Success` or `Failure` randomly.
- `remote`: Connects to a remote server and returns the result. Details are in the [Remote actions](./r_actions.md#remote-actions) documentation.

**Parameters:**
- The `success`, `failure`, and `random` stubs accept a `delay` parameter (in milliseconds).
- The `remote` stub requires a `url` parameter and accepts an optional `server` parameter for Blackboard access.

## Process

You can perform a simulation via the CLI or directly in Rust code.

### In the Console

Use the `f-tree` CLI to run a simulation:

```shell
f-tree sim --root tree/tests/simulator/smoke/ --profile sim.yaml
```

**CLI Defaults:**
- `--root`: If omitted, defaults to the current working directory (`<pwd>`).
- `--main`: If omitted, defaults to `main.tree`.
- `--tree`: Can be omitted if there is only one root definition in the file.
- `--profile`: If omitted, the default success-stub profile is used.

### In the Code

Use `SimulatorBuilder` from the `simulator` module to configure and run the simulation programmatically.

**From the file system:**
```rust
fn smoke() {
    let mut sb = SimulatorBuilder::new();
    let root = PathBuf::from("simulator/smoke");

    sb.root(root.clone());
    sb.profile(PathBuf::from("sim.yaml"));
     
    let mut fb = ForesterBuilder::from_file_system();
    fb.main_file("main.tree".to_string());
    fb.root(root);

    sb.forester_builder(fb);
     
    let mut sim = sb.build().unwrap();
    sim.run().unwrap();
}
```

**From raw text:**
```rust
fn smoke_from_text() {
    let mut sb = SimulatorBuilder::new();
    let sim = PathBuf::from("simulator/smoke/sim.yaml");
    
    sb.profile(sim);
     
    let mut fb = ForesterBuilder::from_text();
    fb.text(r#"
        import "std::actions"

        root main sequence {
            store("info1", "initial")
            retryer(task(config = obj), success())
            store("info2", "finish")
        }

        fallback retryer(t: tree, default: tree) {
            retry(5) t(..)
            fail("just should fail")
            default(..)
        }

        impl task(config: object);
    "#.to_string());    
    
    sb.forester_builder(fb);
    
    let mut sim = sb.build().unwrap();
    sim.run().unwrap();
}
```