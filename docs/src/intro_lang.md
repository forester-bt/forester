# Forester Tree Language (`.tree`)

The Forester Tree Language is a strongly-typed, functional domain-specific language (DSL) designed to describe behavior tree architecture cleanly without verbosity or repetition.

Unlike traditional XML-based or JSON-based behavior tree formats, Forester treats behavior trees as **first-class, composable abstractions**.

---

## Why a Dedicated DSL?

Standard behavior tree formats (such as XML trees used in ROS) often suffer from extreme duplication and verbosity. Forester's DSL solves this by introducing functional primitives:

* **Higher-Order Trees (HOT)**: Pass trees as parameters to other trees to create reusable patterns (e.g., generic retry loops, fallback wrappers, logging decorators).
* **Strong Type System**: Built-in support for strings, numbers, booleans, arrays, objects, and subtrees, validated at compile time.
* **Blackboard Pointers**: Pass live references to Blackboard memory into actions, allowing nodes to operate on dynamically updated state (essential for AI agent context).
* **Lambdas**: Write inline, anonymous subtrees without declaring throwaway named definitions.

---

## Project Structure & File Conventions

Forester projects consist of one or more `.tree` files organized within a root project directory:

```text
my_project/
├── main.tree           # Entry point containing the root tree
├── agent_tools.tree    # Custom actions and tool definitions
├── navigation/
│   ├── move.tree       # Sub-tree definitions
│   └── recovery.tree   # Recovery patterns
```

### Key Conventions

1. **File Extension**: All Forester source files use the `.tree` extension.
2. **Root Entry Point**: A project must define at least one `root` node (e.g., `root main sequence { ... }`).
3. **Module Imports**: Imports resolve relative to the root directory (e.g., `import "navigation::move"`).

---

## Language Roadmap & Tooling

To ensure a seamless developer experience, Forester provides language tooling across popular editors:
* **Syntax Highlighting & Linting**: Editor support for `.tree` files.
* **Language Server Protocol (LSP)**: Autocompletion, inline type errors, and navigation support for VS Code, Neovim, and IntelliJ.
