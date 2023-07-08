# Tracing
By default the engine tries to trace the information about the tree.
It can be helpfull in analysing.

Below, the example how it can be shown in a text form:
```text
[1]  1 : Running(cursor=0,len=1)
[1]    2 : Running(cursor=0,len=3)
[1]      3 : Success(key=x,value=tick)
[1]    2 : Running(cursor=1,len=3)
[1]      4 : Success(name=tick)
[1]    2 : Running(cursor=2,len=3)
[1]      5 : Running(cursor=0,len=2)
[1]        6 : Success(k=a,i=1)
[1]      5 : Running(cursor=1,len=2)
[1]        7 : Running(cursor=0,len=2)
[1]          8 : Failure(key=x,expected=10,reason=1 != 10)
[1]        7 : Running(cursor=1,len=2)
[1]          9 : Running()
[1]        7 : Running(cursor=1,len=2)
[1]      5 : Running(cursor=1,len=2,prev_cursor=1)
[1]    2 : Running(cursor=2,len=3)
[2]  next tick
[2]    2 : Running(cursor=0,len=3)
[2]      3 : Success(key=x,value=tick)
[2]    2 : Running(cursor=1,len=3)
[2]      4 : Success(name=tick)
[2]    2 : Running(cursor=2,len=3)
[2]      5 : Running(cursor=0,len=2,prev_cursor=1)
[2]        7 : Running(cursor=0,len=2)
[2]          8 : Failure(key=x,expected=10,reason=2 != 10)
[2]        7 : Running(cursor=1,len=2)
[2]          9 : Running()
[2]        7 : Running(cursor=1,len=2)
[2]      5 : Running(cursor=0,len=2,prev_cursor=1)
[2]    2 : Running(cursor=2,len=3)
[2]  1 : Running(cursor=0,len=1)
```
The first symbol `[X]` denotes the current tick. 
The indent shows the level of nesting.
Next it is a pairt of node id and the status with parameters.

## Custom messages
The users can add the custom messages using the parameter `Tracer` from context:

```f-tree
impl custom_state();
root main repeat(3) custom_state()
```


```rust
    struct CT;

    impl Impl for CT {
        fn tick(&self, args: RtArgs, ctx: &mut TreeContext) -> Tick {
            let i = ctx
                .bb()
                .get("k".to_string())?
                .and_then(|v| v.clone().as_int())
                .map(|v| v + 1)
                .unwrap_or_default();

            ctx.bb().put("k".to_string(), RtValue::int(i));
            // the method trace accepts Event::Custom
            ctx.trace(Event::Custom(format!("i = {:?}", i)));
            
            Ok(TickResult::success())
        }
    }

```

That will igve the following trace:

```
[1]  1 : Running(cursor=0,len=1)
[1]    2 : Running(len=1)
[1]      i = 0
[1]      3 : Success()
[1]    2 : Running(arg=2,cursor=0,len=1)
[2]  next tick
[2]    2 : Running(arg=2,cursor=0,len=1)
[2]      i = 1
[2]      3 : Success()
[2]    2 : Running(arg=3,cursor=0,len=1)
[2]  1 : Running(cursor=0,len=1)
[3]  next tick
[3]    2 : Running(arg=3,cursor=0,len=1)
[3]      i = 2
[3]      3 : Success()
[3]    2 : Success(arg=3,cursor=0,len=1)
[3]  1 : Running(cursor=0,len=1)
[3]  1 : Success(cursor=0,len=1)
```

## Configuration

The tracer has a few settings.
- indent: the indent of the lines in dependance of the nesting level
- to_file: if the file is provided, the trace will be duplicated into this file also.

```rust
#[test]
fn file() {
    let mut fb = fb("tracer/custom");
    let tracer_log = test_folder("tracer/custom/main.trace");

    fb.tracer(Tracer::create(TracerConfiguration {
        indent: 2,
        to_file: Some(tracer_log.clone()),
    }));

    let mut f = fb.build().unwrap();
    let result = f.start();
    assert_eq!(result, Ok(TickResult::success()));

    let file_trace = fs::read_to_string(tracer_log).unwrap();
    assert_eq!(file_trace, f.tracer.to_string())
}
```