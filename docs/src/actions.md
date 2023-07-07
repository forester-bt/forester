# Actions

The leaves of the tree are actions, which are the final point of the whole execution mechanism.
They are supposed to be implemented either using `rust` or other languages
and mix-in on the execution stage.

The actions have 2 keywords to mark:

- `impl` means some job or action that can take time and be asynchronous
- `cond` means some activity to check some conditions and swiftly returns result

*In practice, the engine does not see difference*

in the language, they can be marked as an operation with empty or lacking implementation.

```f-tree
impl action1(a:string); // here the semicolon is required
impl action2(b:object){} // here, not

cond cond1(c:num);
cond cond2(d:array){}

```

## Contract
**The contract of the definition and invocation should coincide, otherwise the execution will throw an exception** 

```f-tree
impl action(c:num)

root main action() // the exception will be raised since the argument is uncovered.
```