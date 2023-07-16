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
    Pointer(BBKey),
    Call(Call),
}
```

### RtValueCast

The helper to quickly cast to the internal type.

```rust
fn test(v:RtValue) {
    let val = v.cast(ctx.bb()?).string()?.unwrap_or_default();
}
```