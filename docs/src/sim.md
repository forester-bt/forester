# Simulation

Forester provides a conception to execute the given tree, replacing the actions to stubs.
The major intention is to test the tree behaviour and test all branches under the specific conditions
without coding the implementations at all.
The profile enables to mix in the specific state of blackboard, trace the changes and visualize the tree.


## Preparations


### Configuration profile

**All paths in the configuration files can be either absolute or relative to the root folder**

The file contains the settings information alongside with the information about stubbed options.

Below, the example of the file:

```yaml
config:
  trace: gen/main.trace
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

Config section:

| Setting   | Description                                      | Default                                  | Example          |
|-----------|--------------------------------------------------|------------------------------------------|------------------|
| trace     | the trace file                                   | if it is absent, there will be no action | gen/main.trace   |
| graph     | the visualization file                           | if it is absent, there will be no action | gen/main.svg     |
| bb.dump   | the dump of the bb at the end                    | if it is absent, there will be no action | gen/bb.json      |
| bb.load   | the dump that will be used to init bb before sim | if it is absent, there will be no action | gen/init_bb.json |
| max_ticks | the maximum amount of ticks to work.             | 0 by default                             | 10               |

Actions sections:

The actions sections is an array to stub the actions

| Setting      | Description                              | Default             | Example |
|--------------|------------------------------------------|---------------------|---------|
| name         | the name of the stubbed action           | should be presented | name    |
| stub         | the stubbed implementation               | should be presented | success |
| params.delay | denotes the pause before start in millis | 0                   | 100     |


### Stubs

- success: returns a success
- failure: returns a failure
- random: returns either a failure or a success randomly

All stubs have the following params:
- delay: in millis, the time to delay the stub.

## Process

The simulation can be performed in on of two ways:
- using console application from console
- using a library from rust code

### In the code

Just use the builder and the simulator from the `simulator` module. For details, 
please see the doc for `ForesterBuilder` and `SimulatorBuilder`

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

 fn smoke_from_text() {
     let mut sb = SimulatorBuilder::new();

     let sim = PathBuf::from("simulator/smoke/sim.yaml");
     let mut fb = ForesterBuilder::from_text();
     sb.profile(sim);
     
     fb.text(
         r#"
 import "std::actions"

 root main sequence {
     store_str("info1", "initial")
     retryer(task(config = obj), success())
     store_str("info2","finish")
 }

 fallback retryer(t:tree, default:tree){
     retry(5) t(..)
     fail("just should fail")
     default(..)
 }

 impl task(config: object);
     "#
         .to_string(),
     );    
     sb.forester_builder(fb);
     let mut sim = sb.build().unwrap();
     sim.run().unwrap();
 }

```


### In the console

Just use a `f-tree` console cli to run a simulation

```shell
f-tree sim --root tree\tests\simulator\smoke\  --profile sim.yaml
```

- root can be ommited, the `<pwd>` folder will be taken by default
- tree can be ommited if only one root definition in the file
- main can be ommited, by default, the name `main.tree` will be taken.  
