use crate::runtime::action::builtin::data::{GenerateData};
use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::{TreeContextRef};
use crate::runtime::TickResult;
use crate::tests::{fb, test_folder};
use crate::tracer;
use crate::tracer::{Tracer, TracerConfig};

use std::fs;


#[test]
fn smoke() {
    let mut fb = fb("flow/sequence_running");
    fb.tracer(Tracer::default());
    fb.register_sync_action(
        "incr",
        GenerateData::new(|v| {
            let curr = v.as_int().unwrap_or(0);
            RtValue::int(curr + 1)
        }),
    );

    let mut f = fb.build().unwrap();
    let result = f.run();
    assert_eq!(result, Ok(TickResult::success()));

    let trace = f.tracer.lock().unwrap().to_string();
    assert_eq!(
        trace,
        r#"[1]  1 : Running(cursor=0,len=1)
[1]    2 : Running(cursor=0,len=3)
[1]      3 : Success(key=x,value=tick)
[1]    2 : Running(cursor=1,len=3)
[1]      4 : Success(name=&x)
[1]    2 : Running(cursor=2,len=3)
[1]      5 : Running(cursor=0,len=2)
[1]        6 : Success(k=a,i=1)
[1]      5 : Running(cursor=1,len=2)
[1]        7 : Running(cursor=0,len=2)
[1]          8 : Failure(key=&tick,expected=10,reason=1 != 10)
[1]        7 : Running(cursor=1,len=2)
[1]          9 : Running()
[1]        7 : Running(cursor=1,len=2)
[1]      5 : Running(cursor=1,len=2,prev_cursor=1)
[1]    2 : Running(cursor=2,len=3)
[2]  next tick
[2]    2 : Running(cursor=0,len=3)
[2]      3 : Success(key=x,value=tick)
[2]    2 : Running(cursor=1,len=3)
[2]      4 : Success(name=&x)
[2]    2 : Running(cursor=2,len=3)
[2]      5 : Running(cursor=0,len=2,prev_cursor=1)
[2]        7 : Running(cursor=0,len=2)
[2]          8 : Failure(key=&tick,expected=10,reason=2 != 10)
[2]        7 : Running(cursor=1,len=2)
[2]          9 : Running()
[2]        7 : Running(cursor=1,len=2)
[2]      5 : Running(cursor=0,len=2,prev_cursor=1)
[2]    2 : Running(cursor=2,len=3)
[2]  1 : Running(cursor=0,len=1)
[3]  next tick
[3]    2 : Running(cursor=0,len=3)
[3]      3 : Success(key=x,value=tick)
[3]    2 : Running(cursor=1,len=3)
[3]      4 : Success(name=&x)
[3]    2 : Running(cursor=2,len=3)
[3]      5 : Running(cursor=0,len=2,prev_cursor=1)
[3]        7 : Running(cursor=0,len=2)
[3]          8 : Failure(key=&tick,expected=10,reason=3 != 10)
[3]        7 : Running(cursor=1,len=2)
[3]          9 : Running()
[3]        7 : Running(cursor=1,len=2)
[3]      5 : Running(cursor=0,len=2,prev_cursor=1)
[3]    2 : Running(cursor=2,len=3)
[3]  1 : Running(cursor=0,len=1)
[4]  next tick
[4]    2 : Running(cursor=0,len=3)
[4]      3 : Success(key=x,value=tick)
[4]    2 : Running(cursor=1,len=3)
[4]      4 : Success(name=&x)
[4]    2 : Running(cursor=2,len=3)
[4]      5 : Running(cursor=0,len=2,prev_cursor=1)
[4]        7 : Running(cursor=0,len=2)
[4]          8 : Failure(key=&tick,expected=10,reason=4 != 10)
[4]        7 : Running(cursor=1,len=2)
[4]          9 : Running()
[4]        7 : Running(cursor=1,len=2)
[4]      5 : Running(cursor=0,len=2,prev_cursor=1)
[4]    2 : Running(cursor=2,len=3)
[4]  1 : Running(cursor=0,len=1)
[5]  next tick
[5]    2 : Running(cursor=0,len=3)
[5]      3 : Success(key=x,value=tick)
[5]    2 : Running(cursor=1,len=3)
[5]      4 : Success(name=&x)
[5]    2 : Running(cursor=2,len=3)
[5]      5 : Running(cursor=0,len=2,prev_cursor=1)
[5]        7 : Running(cursor=0,len=2)
[5]          8 : Failure(key=&tick,expected=10,reason=5 != 10)
[5]        7 : Running(cursor=1,len=2)
[5]          9 : Running()
[5]        7 : Running(cursor=1,len=2)
[5]      5 : Running(cursor=0,len=2,prev_cursor=1)
[5]    2 : Running(cursor=2,len=3)
[5]  1 : Running(cursor=0,len=1)
[6]  next tick
[6]    2 : Running(cursor=0,len=3)
[6]      3 : Success(key=x,value=tick)
[6]    2 : Running(cursor=1,len=3)
[6]      4 : Success(name=&x)
[6]    2 : Running(cursor=2,len=3)
[6]      5 : Running(cursor=0,len=2,prev_cursor=1)
[6]        7 : Running(cursor=0,len=2)
[6]          8 : Failure(key=&tick,expected=10,reason=6 != 10)
[6]        7 : Running(cursor=1,len=2)
[6]          9 : Running()
[6]        7 : Running(cursor=1,len=2)
[6]      5 : Running(cursor=0,len=2,prev_cursor=1)
[6]    2 : Running(cursor=2,len=3)
[6]  1 : Running(cursor=0,len=1)
[7]  next tick
[7]    2 : Running(cursor=0,len=3)
[7]      3 : Success(key=x,value=tick)
[7]    2 : Running(cursor=1,len=3)
[7]      4 : Success(name=&x)
[7]    2 : Running(cursor=2,len=3)
[7]      5 : Running(cursor=0,len=2,prev_cursor=1)
[7]        7 : Running(cursor=0,len=2)
[7]          8 : Failure(key=&tick,expected=10,reason=7 != 10)
[7]        7 : Running(cursor=1,len=2)
[7]          9 : Running()
[7]        7 : Running(cursor=1,len=2)
[7]      5 : Running(cursor=0,len=2,prev_cursor=1)
[7]    2 : Running(cursor=2,len=3)
[7]  1 : Running(cursor=0,len=1)
[8]  next tick
[8]    2 : Running(cursor=0,len=3)
[8]      3 : Success(key=x,value=tick)
[8]    2 : Running(cursor=1,len=3)
[8]      4 : Success(name=&x)
[8]    2 : Running(cursor=2,len=3)
[8]      5 : Running(cursor=0,len=2,prev_cursor=1)
[8]        7 : Running(cursor=0,len=2)
[8]          8 : Failure(key=&tick,expected=10,reason=8 != 10)
[8]        7 : Running(cursor=1,len=2)
[8]          9 : Running()
[8]        7 : Running(cursor=1,len=2)
[8]      5 : Running(cursor=0,len=2,prev_cursor=1)
[8]    2 : Running(cursor=2,len=3)
[8]  1 : Running(cursor=0,len=1)
[9]  next tick
[9]    2 : Running(cursor=0,len=3)
[9]      3 : Success(key=x,value=tick)
[9]    2 : Running(cursor=1,len=3)
[9]      4 : Success(name=&x)
[9]    2 : Running(cursor=2,len=3)
[9]      5 : Running(cursor=0,len=2,prev_cursor=1)
[9]        7 : Running(cursor=0,len=2)
[9]          8 : Failure(key=&tick,expected=10,reason=9 != 10)
[9]        7 : Running(cursor=1,len=2)
[9]          9 : Running()
[9]        7 : Running(cursor=1,len=2)
[9]      5 : Running(cursor=0,len=2,prev_cursor=1)
[9]    2 : Running(cursor=2,len=3)
[9]  1 : Running(cursor=0,len=1)
[10]  next tick
[10]    2 : Running(cursor=0,len=3)
[10]      3 : Success(key=x,value=tick)
[10]    2 : Running(cursor=1,len=3)
[10]      4 : Success(name=&x)
[10]    2 : Running(cursor=2,len=3)
[10]      5 : Running(cursor=0,len=2,prev_cursor=1)
[10]        7 : Running(cursor=0,len=2)
[10]          8 : Success(key=&tick,expected=10)
[10]        7 : Success(cursor=0,len=2)
[10]      5 : Success(cursor=1,len=2,prev_cursor=1)
[10]    2 : Success(cursor=2,len=3)
[10]  1 : Running(cursor=0,len=1)
[10]  1 : Success(cursor=0,len=1)
"#
        .replace("\n", tracer::LINE_ENDING)
    )
}

#[test]
fn custom_state() {
    let mut fb = fb("tracer/custom");
    fb.tracer(Tracer::default());
    struct CT;

    impl Impl for CT {
        fn tick(&self, _args: RtArgs, ctx: TreeContextRef) -> Tick {
            let i = ctx
                .bb()
                .lock()
                .unwrap()
                .get("k".to_string())?
                .and_then(|v| v.clone().as_int())
                .map(|v| v + 1)
                .unwrap_or_default();

            ctx.bb()
                .lock()
                .unwrap()
                .put("k".to_string(), RtValue::int(i))
                .unwrap();
            ctx.trace(format!("i = {:?}", i)).unwrap();
            Ok(TickResult::success())
        }
    }

    fb.register_sync_action("custom_state", CT);

    let mut f = fb.build().unwrap();
    let result = f.run();
    assert_eq!(result, Ok(TickResult::success()));

    let trace = f.tracer.lock().unwrap().to_string();
    assert_eq!(
        trace,
        r#"[1]  1 : Running(cursor=0,len=1)
[1]    2 : Running(len=1)
[1]      custom: i = 0
[1]      3 : Success()
[1]    2 : Running(arg=2,cursor=0,len=1)
[2]  next tick
[2]    2 : Running(arg=2,cursor=0,len=1)
[2]      custom: i = 1
[2]      3 : Success()
[2]    2 : Running(arg=3,cursor=0,len=1)
[2]  1 : Running(cursor=0,len=1)
[3]  next tick
[3]    2 : Running(arg=3,cursor=0,len=1)
[3]      custom: i = 2
[3]      3 : Success()
[3]    2 : Success(arg=3,cursor=0,len=1)
[3]  1 : Running(cursor=0,len=1)
[3]  1 : Success(cursor=0,len=1)
"#
        .replace("\n", tracer::LINE_ENDING)
    )
}

#[test]
fn file() {
    let mut fb = fb("tracer/custom");
    struct CT;

    impl Impl for CT {
        fn tick(&self, _args: RtArgs, ctx: TreeContextRef) -> Tick {
            let i = ctx
                .bb()
                .lock()
                .unwrap()
                .get("k".to_string())?
                .and_then(|v| v.clone().as_int())
                .map(|v| v + 1)
                .unwrap_or_default();

            ctx.bb()
                .lock()
                .unwrap()
                .put("k".to_string(), RtValue::int(i))
                .unwrap();
            ctx.trace(format!("i = {:?}", i)).unwrap();
            Ok(TickResult::success())
        }
    }

    fb.register_sync_action("custom_state", CT);

    let tracer_log = test_folder("tracer/custom/main.trace");

    fb.tracer(
        Tracer::create(TracerConfig {
            indent: 2,
            time_format: None,
            to_file: Some(tracer_log.clone()),
        })
        .unwrap(),
    );

    let mut f = fb.build().unwrap();
    let result = f.run();
    assert_eq!(result, Ok(TickResult::success()));

    let file_trace = fs::read_to_string(tracer_log.clone()).unwrap();
    fs::remove_file(tracer_log).unwrap();

    assert_eq!(
        file_trace,
        r#"[1]  1 : Running(cursor=0,len=1)
[1]    2 : Running(len=1)
[1]      custom: i = 0
[1]      3 : Success()
[1]    2 : Running(arg=2,cursor=0,len=1)
[2]  next tick
[2]    2 : Running(arg=2,cursor=0,len=1)
[2]      custom: i = 1
[2]      3 : Success()
[2]    2 : Running(arg=3,cursor=0,len=1)
[2]  1 : Running(cursor=0,len=1)
[3]  next tick
[3]    2 : Running(arg=3,cursor=0,len=1)
[3]      custom: i = 2
[3]      3 : Success()
[3]    2 : Success(arg=3,cursor=0,len=1)
[3]  1 : Running(cursor=0,len=1)
[3]  1 : Success(cursor=0,len=1)
"#
        .replace("\n", tracer::LINE_ENDING)
    );
}
