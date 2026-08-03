![Logo](pics/logo.png)

# Forester

**Forester** is a domain-specific language (DSL) and high-performance Rust runtime designed for building, testing, and executing **behavior trees**.

It is specifically engineered for two core domains:
- **🧠 AI Agents**: Orchestrating LLM-based tools, state management, and fallback loops without unmaintainable Python code or DAG spaghettification.
- **🤖 Robotics & Industrial Systems**: Managing deterministic, reactive hardware control loops, with built-in export to **ROS Nav2** and simulation support for **Webots**.

---

## Why Forester?

As systems grow in complexity, state machines and Directed Acyclic Graphs (DAGs) break down when handling fallbacks, retries, and dynamic error recovery. Forester separates **decision logic** (tree orchestration) from **execution logic** (actions), enabling modular, maintainable control flow.

### Key Differentiators

* **Dedicated Orchestration DSL**: Write tree logic once in a strongly-typed, functional language featuring higher-order trees, lambdas, and live Blackboard pointers.
* **Sync & Async Execution**: Non-blocking async execution for I/O and remote calls, alongside predictable sync execution for tight control loops.
* **Local & Remote Action Clients**: Execute actions natively in Rust (`forester-rs`) or remotely via lightweight clients (such as Python's `forester-http-ra-py` for LangChain/LlamaIndex tools).
* **Rich Analysis & Simulation**: Inspect execution traces, visualize trees, and test orchestration in simulation before deploying to physical hardware or production agents.
* **ROS Nav2 & Webots Integration**: Export Forester trees directly into ROS Nav2 XML or run against Webots simulation environments.
* **Static Validations & Optimizations**: Catch tree errors, numeric overflows, and structural flaws at compile time rather than runtime.

---

## Why Behavior Trees?

Behavior trees provide a clean, mathematical abstraction over decision logic:
* **Modularity**: Small, independent nodes (sequences, fallbacks, decorators) can be composed into deep hierarchical behaviors.
* **Reactivity**: Ticks evaluate preconditions dynamically, allowing real-time preemptions and fallbacks when environment state changes.
* **Separation of Concerns**: Tree nodes decide *what* to do next; external action handlers perform *how* it gets done.

### Learn More About Behavior Trees
- [Chris Simpson’s Behavior Trees for AI](https://outforafight.wordpress.com/2014/07/15/behaviour-behavior-trees-for-ai-dudes-part-1/)
- [Introduction to Behavior Trees](https://robohub.org/introduction-to-behavior-trees/)
- [State Machines vs. Behavior Trees](https://www.polymathrobotics.com/blog/state-machines-vs-behavior-trees)

### Related Ecosystem Projects
- [BehaviorTree.CPP](https://www.behaviortree.dev/) — C++ behavior tree framework widely used in ROS2.
- [Beehave](https://github.com/bitbrain/beehave) — Behavior tree AI library for the Godot Engine.
- [Bonsai](https://github.com/Sollimann/bonsai) — Pure Rust behavior tree implementation.
