# Runtime arguments

The runtime representation of the static arguments from the code. 
It has the same set of types and easily transforms one to another

```rust
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum RtValue {
    String(String),
    Bool(bool),
    Array(Vec<RtValue>),
    Object(HashMap<String, RtValue>),
    Number(RtValueNumber),
    Pointer(BBKey)
    ...
}
```

### Primitive types
The types `String`, `Bool`, `Number` are primitive types. They act as their counterparts in the other languages

### Complex types
The types `Object` acts as a json map and `Array` just as an array.

### Pointer
Represents a name of the cell in bb. For example, in the expression `eq_num(tick, 10)` the tick is a pointer and represents 
a name of the cell where the value is stored. Thus, the action will go to the cell `tick` and extract the value and compare 
it with number.


```f-tree
import "std::actions"
impl incr(k:string, i:num);

root main r_sequence{
    store_tick("tick")
    sequence {
        r_fallback {
            eq_num(tick, 10)
            running()
        }
    }
}

```

Remark. 
The action `store_tick("tick")` accepts a cell name as a string,
otherwise if it is a pointer the action will be looking a name inside the cell with the name `tick`.
The example depicts it:

```
    store_str("x","tick")
    store_tick(x)
    eq_num(tick, 10)
```

### How to work with arguments

There are two ways to extract the values from the arguments:

#### Directly using as_<type> method
Every argument can be immediately converted to the specific primitive or complex type.

```rust
fn to(v:RtValue) {
    let val:Option<String> = v.as_string();
}
```

This is the cheapest way to do it. But also, this way does not consider pointers, 
therefore, it can be used only if you are sure the type is primitive or complex. 

#### Using cast(ctx) method

The method accepts context for being able to unfold the pointer if it is presents.


```rust
impl Impl for CheckEq {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let key = args
            .find_or_ith("key".to_string(), 0)
            .ok_or(RuntimeError::fail(format!("the key is expected ")))?;
        
        // cast accepts a context to be able to resolve pointers if they are presented
        let param = key.cast(ctx.clone()).str()?;
    }
}
```

This is method preferable if you are not sure what can come in the arguments.