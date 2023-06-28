# Flow tree definitions

The flow tree definitions describe the way how the tree will be traversed.
There are 2 basic types which get broken down afterward:
- sequences: A Sequence performs every child as long as they are return `Success`.
If otherwise, the sequence instantly stops with the `failure` status
- fallbacks: A fallback performs children until the first `Success`.

Combining the aforementioned flow trees, we can get any type of logic.


