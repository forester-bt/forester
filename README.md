<p align="center">
    <img width="255" alt="Logo" src="docs/src/pics/logo.png">
</p>
<h1 align="center">Forester - A fast and modular orchestration engine, implementing behavior trees.</h1>


<p align="center">
  <img alt="Visualization of the tree"
       src="docs/src/pics/main.svg">
</p>

## About

Forester represents a framework that provides the toolkit to perform the effective task orchestrations.
The tasks can be performed synchronously or asynchronously, locally or remotely.
Forester takes care of the correct performance and distribution of the tasks.
The main concept of the framework is the flow based on the behavior trees
it can be effectively used in the game, ai, robotic areas, or anywhere where the workflow engine can be applied.

## Example

The language is very simple
```
import "std::actions"

root main sequence {
    store("info", "initial")
    retryer(task({}), success())
    store("field","some_info")
}

fallback retryer(t:tree, default:tree){
    retry(5) t(..)
    fail("just should fail")
    default(..)
}

impl task(config: object);
```

and simulator test

```rust
#[test]
fn smoke() {
    let mut sb = SimulatorBuilder::new();

    let root = test_folder("simulator/smoke");

    sb.root(root);
    sb.profile(PathBuf::from("sim.yaml"));
    sb.main_file("main.tree".to_string());

    let mut sim = sb.build().unwrap();
    let tracer = &sim.forester.tracer;
    sim.run().unwrap();
}
```

or running from the console

```shell
forest sim --root tree\tests\simulator\smoke\  --profile sim.yaml
```

## Why Forester

The main idea and the target of Forester is to make the process of chaining a complex logic
of the different tasks together effective and easy.

The following set of features is summoned to highlight the framework among the others.

- The dsl to describe the logic
- The framework provides the ability to create async and sync tasks 
- The framework provides the ability to create remote and local tasks
- The tooling to visualize and trace the execution of the tree
- The simulation mode is supposed to aid with the design decisions
- The optimizations and analysis of the tree
- The validation engine allows the users to create the manually defined validations

## Why behavior trees

Firstly, they provide a strong math abstraction over the orchestration logic \
and enables to separate the business logic and the tree logic itself.

On the other hand, they have only a small set of logically conjucted components making the design easier,
One of the great advantages of the behavior trees is that they provide a good conception of modularity. \
Articles that introduce into the basics of the behavior trees
- [Chris Simpson’s Behavior trees for AI: How they work](https://outforafight.wordpress.com/2014/07/15/behaviour-behavior-trees-for-ai-dudes-part-1/)
- [Introduction to behavior trees](https://robohub.org/introduction-to-behavior-trees/)
- [State Machines vs Behavior Trees](https://www.polymathrobotics.com/blog/state-machines-vs-behavior-trees)
 
## Documentation

The common documentation is available as a [mini-book](https://forester-bt.github.io/forester/), describing all aspects of working with the framework

## Publications
 - [Part I. The orchestration with behavior trees. Simulation.](https://medium.com/@zhguchev/forester-the-orchestration-with-behaviour-trees-part-i-simulation-b10867aab8db)
 - [Part II. The dsl above trees. Higher order trees.](https://medium.com/@zhguchev/forester-part-ii-why-do-we-need-to-have-a-language-above-trees-bdf046bf4a73)
 - [Part III. How to change the runtime tree on the fly. Trimming.](https://medium.com/@zhguchev/forester-part-iii-trimming-change-the-runtime-tree-on-the-fly-185a6e61a7aa)

## Contributing

Currently, the project is under active development (there are a lot of blind spots), so any help is highly appreciated.
Any help is highly appreciated at any stage of the project but at now especially.

A guideline about contributing to Forester can be found in the
[`CONTRIBUTING.md`](CONTRIBUTING.md) file.

## License

Forester is released under the [Apache License, Version 2.0](LICENSE).

## Logo

<a href="https://www.freepik.com/free-vector/logo-with-abstract-tree_29192741.htm#from_view=detail_alsolike">The logo provided by bunny</a> on Freepik


