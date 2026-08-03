# Forester Architecture & Components

Forester is structured into four primary subsystems: the **Language & Compiler**, the **Runtime Engine**, **Analysis & Simulation**, and **Integrations & Tooling**.

---

## 1. Language & Compiler

Forester provides a domain-specific language (DSL) tailored for behavior tree orchestration.

* **Tree DSL (`.tree`)**: A strongly-typed language for describing execution logic, higher-order trees, lambdas, decorators, and Blackboard memory references.
* **Static Analysis & Compiler**: Parses scripts, verifies type constraints, validates argument bounds, and detects structural errors before runtime.

---

## 2. Runtime Engine

The Rust runtime engine manages tree execution and state.

* **Engine Core**: Ticks nodes reactively, supporting synchronous execution (for fast control loops) and asynchronous execution (for non-blocking I/O).
* **Blackboard**: High-performance, shared memory store for passing data between tree nodes dynamically.
* **Action Keeper**: Registers and executes actions (local Rust callbacks or remote RPC handlers).

---

## 3. Analysis & Simulation

Tools to inspect, debug, and validate behavior trees during development.

* **`f-tree` CLI**: Unified command-line tool for compiling, analyzing, and running trees.
* **Visualization**: Generates tree diagrams (Graphviz) to inspect tree structure visually.
* **Execution Tracing**: Detailed execution telemetry for stepping through ticks and debugging node state.
* **Simulator**: Executes trees against stubbed action responses to test orchestration logic before linking real hardware or external LLM APIs.

---

## 4. Integrations & Tooling

Bridge Forester to external runtimes, robotics engines, and developer editors.

* **Remote Action SDKs**: Lightweight clients allowing actions to run outside the Rust process:
  - **Python (`forester-http-ra-py`)**: Execute Python functions and LangChain/LlamaIndex tools seamlessly over HTTP.
  - **Rust (`forester-rs`)**: Native Rust action integration.
* **ROS Nav2 Exporter**: Exports Forester trees directly to ROS2 Nav2 XML format for robotics navigation pipelines.
* **Editor Tooling**: Syntax highlighting, linting, and Language Server Protocol (LSP) support for IDEs (VS Code, IntelliJ).
