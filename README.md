<p align="center">
    <img width="255" alt="Logo" src="assets/logo.png">
</p>

<h1 align="center">Forester</h1>
<p align="center"><b>A functionally-oriented language and a fast, typed runtime for behavior trees — built to orchestrate robots and AI agents alike.</b></p>

<p align="center">
  <img alt="Visualization of the tree" src="assets/main.svg">
</p>

## What is Forester

Forester is a language *and* a Rust runtime for describing and executing **behavior trees**.
If you are orchestrating LLM-based agents or reactive hardware, standard DAGs (Directed Acyclic Graphs) 
and state machines quickly degrade into unmaintainable spaghetti code when handling fallbacks, retries, and error recovery. 
Behavior trees give you a small, well-understood set of composable primitives (sequence, fallback, decorator) with clean modularity.

A behavior tree separates *what your system should decide* from *how each step gets done*. 
Forester makes that separation cheap: you write the decision logic once in a real language with types and reusable abstractions, 
and you plug in whatever does the work (a motor controller, a REST call, or an LLM completion).

Tasks run **sync or async**, **local or remote**, **sequentially or in parallel**. 
Forester handles the scheduling, retries, timeouts, and distribution, keeping your orchestration logic about the *logic*, not the plumbing.

## Installation
Add the runtime to your Rust project:
```shell
cargo add forester-rs
```
Install the CLI tool for analysis, simulation, and tracing:
```shell
cargo install f-tree
```


## Target Domains

The same core engine and DSL are already used for two quite different jobs, and that's deliberate:

**🤖 Robotics & industrial systems**
Deterministic, safety-critical control flow with real hardware in the loop.
Forester ticks reactively, exports to **ROS Nav2**, and integrates with simulators like Webots,
so you can validate a tree against a simulated robot before it ever touches hardware.

**🧠 AI agents**
Structured, inspectable orchestration for LLM-driven systems as an alternative to hand-rolled if/else chains. 
A tool call or LLM response lands on the Blackboard, and the next node reads it by reference. 
Because the tree is a real artifact, you get retries, fallbacks, and tracing for free.

*Python developers*: You do not need to write Rust to use Forester. 
Use the forester-http-ra-py client to execute your Python LangChain/LlamaIndex tools via HTTP 
while Forester handles the state and orchestration.

## A taste of the language

```
import "std::actions"

root main sequence {
    // 1. Check preconditions
    fallback {
        battery_ok()
        navigate_to_charger()
    }
    
    // 2. Execute complex logic using a higher-order tree
    retryer(
        call_llm({"prompt": "analyze_scene"}), 
        alert_operator()
    )
}

// Write the fallback/retry pattern once, reuse it everywhere.
fallback retryer(t: tree, default: tree) {
    retry(3) t(..)
    default(..)
}

// Define the signatures of the actions executed by your runtime
impl battery_ok();
impl navigate_to_charger();
impl call_llm(config: object);
impl alert_operator();
```

A few things that aren't typical for a behavior-tree DSL:

 * Higher-order trees: retryer takes other trees (t, default) as parameters and invokes them with (..). Write retry/fallback/logging patterns once, reuse them everywhere, no copy-paste.
 * A real type system: numbers, strings, bools, arrays, objects, and trees themselves are all typed, with compile-time checks (including numeric overflow).
 * Pointers into the Blackboard: an argument can be a live reference to shared state, resolved at invocation time rather than captured at definition time. This is what makes the AI-agent integration painless.
 * Lambdas: anonymous, inline subtrees for the cases that don't deserve a name.

### Running the Tree
You define the logic in .tree files, but you execute the actions in your application language. Here is how you bind the call_llm action to the Rust runtime and tick the tree:
```rust
use forester_rs::runtime::action::Action;
use forester_rs::runtime::builder::ForesterBuilder;

// 1. Define your action execution
struct CallLlm;
impl Action for CallLlm {
    fn tick(&self, args: ActionArgs) -> TickResult {
        // Execute your LLM API call here
        TickResult::Success
    }
}

fn main() {
    // 2. Build the runtime and register the action
    let mut runtime = ForesterBuilder::new()
        .main_file("main.tree")
        .register_sync_action("call_llm", CallLlm)
        .build()
        .unwrap();

    // 3. Run the orchestration
    runtime.run().unwrap();
}
```
## What's in the box

| | |
|---|---|
| **Language** | Sequences, fallbacks, parallel, decorators, higher-order trees, lambdas, a typed parameter system |
| **Runtime** | Sync/async and local/remote task execution, parallelization, retries and timeouts, a Blackboard for shared state |
| **Analysis** | Visualization, execution tracing, a simulator for testing trees before deployment |
| **Integrations** | ROS Nav2 export, Webots, remote-action clients ([Rust](https://github.com/forester-bt/forester-rs), [Python](https://github.com/forester-bt/forester-http-ra-py)) |
| **Tooling** | `f-tree` CLI, [IntelliJ plugin](https://github.com/forester-bt/forester-intellij-plugin) |

## Documentation

The full book lives at [forester-bt.github.io/learn](https://forester-bt.github.io/learn/) — language reference, runtime internals, analysis tools, and integrations.

If you are new to behavior trees:
- [Chris Simpson — Behavior trees for AI: how they work](https://outforafight.wordpress.com/2014/07/15/behaviour-behavior-trees-for-ai-dudes-part-1/)
- [State machines vs. behavior trees](https://www.polymathrobotics.com/blog/state-machines-vs-behavior-trees)

## Writing

- [Part I — Orchestration with behavior trees: simulation](https://medium.com/@zhguchev/forester-the-orchestration-with-behaviour-trees-part-i-simulation-b10867aab8db)
- [Part II — A language above trees: higher-order trees](https://medium.com/@zhguchev/forester-part-ii-why-do-we-need-to-have-a-language-above-trees-bdf046bf4a73)
- [Part III — Changing the runtime tree on the fly: trimming](https://medium.com/@zhguchev/forester-part-iii-trimming-change-the-runtime-tree-on-the-fly-185a6e61a7aa)

## Contributing

The project is under active development again, and there's plenty of room to help — from language features to integrations to docs.
See [`CONTRIBUTING.md`](CONTRIBUTING.md).

## License

Apache License, Version 2.0. See [`LICENSE`](LICENSE).

## Logo

Logo by [bunny on Freepik](https://www.freepik.com/free-vector/logo-with-abstract-tree_29192741.htm#from_view=detail_alsolike).