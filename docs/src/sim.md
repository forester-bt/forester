# Simulation

Forester provides a conception to execute the given tree, replacing the actions to stubs.
The major intention is to test the tree behaviour and test all branches under the specific conditions
without coding the implementations at all.
The profile enables to mix in the specific state of blackboard, trace the changes and visualize the tree.


## Preparations


### Configuration profile

**All paths in the configuration files can be either absolute or relative to the root folder**

The file contains the settings information alongisde with the information about stubbed options.

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
| stub         | the stubbed implmenetation               | should be presented | success |
| params.delay | denotes the pause before start in millis | 0                   | 100     |


## Process

The simulation can be performed in on of two ways:
- using console application from console
- using library from rust code

