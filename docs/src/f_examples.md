## Examples

To help you get started quickly, we maintain a dedicated repository containing fully functional examples that demonstrate Forester in action across different domains.

You can find the repository here: **[forester-bt/examples](https://github.com/forester-bt/examples)**

### Implemented Examples

*   **Basic Simulation:** Demonstrates how to set up a tree, define a profile, and run it using the built-in simulator with stubbed actions.
*   **ROS Nav2 Integration:** A robotics orchestration example showing how to structure a tree for navigation and export it to ROS Nav2 XML.
*   **Webots Robot Control:** Demonstrates how to use the Forester-Webots integration layer to control a simulated robot using reactive ticks.
*   **Remote Actions (Python & Rust):** Shows how to connect external logic—such as LLM agent tools or external APIs—using the HTTP remote action clients, keeping the orchestration in Rust while executing logic in Python.
*   **Higher-Order Trees:** Examples of passing subtrees as arguments to create reusable patterns like `retryer` and fallback wrappers.