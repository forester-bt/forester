# IntelliJ Plugin

## Introduction

The Forester IntelliJ Plugin provides native IDE support for writing and testing behavior trees in the `.tree` language. It does not orchestrate tasks itself; rather, it bridges the IDE environment with the Forester runtime and CLI tools.

This allows you to write type-checked behavior trees, navigate complex hierarchical logic, and run simulations with visual SVG outputs directly from your editor, accelerating the development cycle for robotics and AI agent orchestration.

## Installation

1. Open your IntelliJ IDE (IDEA, CLion, PyCharm, etc.).
2. Go to **Settings** (or **Preferences** on macOS) from the main menu.
3. Select **Plugins** from the left-hand menu.
4. Click on the **Marketplace** tab.
5. Search for "Forester".
6. Click **Install** and restart the IDE to activate the plugin.

## Features

### Syntax Highlighting
Provides specialized syntax highlighting for the `.tree` language, making keywords, higher-order trees, decorators, and actions easily distinguishable.

### Code Folding
Allows you to collapse sections of your behavior trees (such as large sequences or fallbacks), making complex task structures readable and easier to navigate.

### Structure View
Displays the hierarchical organization of your behavior trees in the IDE's Structure tool window. You can quickly see parent-child relationships and jump to specific node definitions.

### Task Visualization
Integrates with Forester's graph generation to visually map out your behavior trees. This outputs interactive or static graphical representations (SVGs) of your execution flows.

### Task Simulation
Run and test your behavior trees directly within the IDE using simulation profiles. You can execute stubbed runs to verify the logic and fallback routing without needing the actual application runtime to be active.

## Usage

### Creating a Run Configuration
1. Open the **Run/Debug Configurations** dialog in your IDE.
2. Click the **+** (Add New Configuration) button and select the Forester configuration type.
3. Specify your root folder, main tree file, and simulation profile (if applicable).

### Running a Simulation via the Editor
1. Open the `.tree` file containing your root node.
2. Click the green **Run** icon located in the editor gutter next to the `root` keyword.
3. The IDE will execute the tree simulation and output the trace logs and visual graphs to your configured output directories.

## Links
- [Repository](https://github.com/forester-bt/forester-intellij-plugin)
- [JetBrains Marketplace Plugin Page](https://plugins.jetbrains.com/plugin/22387-forester)