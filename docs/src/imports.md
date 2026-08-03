# Imports & Modularity

Forester allows behavior tree definitions to be split across multiple files and organized into logical sub-modules. Imports make trees modular, reusable, and easy to maintain across large projects.

Imports are typically declared at the top of `.tree` files.

---

## 1. Import Syntax

### Complete File Import
Imports all tree definitions and action signatures from the target file:

```f-tree
import "std::actions"
import "navigation/nav.tree"
import "sensors/battery.tree"
```

### Selective Import with Aliasing
Imports specific definitions from a file and renames them using aliases to prevent naming conflicts:

```f-tree
import "robot_a/vision.tree" {
    detect_object => detect_robot_a_object,
}

import "robot_b/vision.tree" {
    detect_object => detect_robot_b_object,
}
```

---

## 2. Import Paths

Forester supports three types of import paths:

### Relative Paths (Recommended)
Relative paths are evaluated relative to the project root directory:

Given the project directory structure:
```text
my_project/
├── main.tree
└── modules/
    └── navigation/
        └── move.tree
```

In `main.tree`, import `move.tree` using:
```f-tree
import "modules/navigation/move.tree"
```

### Standard Library Imports
Built-in standard library actions and decorators can be imported using standard package specifiers:

```f-tree
import "std::actions"
```

### Absolute Paths
Absolute file paths can be used (primarily for global system modules):

```f-tree
import "/opt/forester/std/common.tree"
```

---

## 3. Resolving Name Conflicts with Aliases

When two separate `.tree` files define sub-trees with identical names, use alias mappings inside curly braces `{ }` to assign unique local names:

```f-tree
// Import 'check_status' from hardware module
import "hardware/status.tree" {
    check_status => check_hardware_status,
}

// Import 'check_status' from LLM agent module
import "agent/status.tree" {
    check_status => check_agent_status,
}

root main sequence {
    check_hardware_status()
    check_agent_status()
}
```

---

## 4. Circular Import Handling

The Forester compiler automatically detects and resolves circular dependency graphs during compilation, raising a clear static compilation error if unresolvable recursive imports occur.