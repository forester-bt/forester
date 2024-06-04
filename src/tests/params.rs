use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs};
use crate::runtime::context::TreeContextRef;
use crate::runtime::{TickResult};

use crate::tests::{fb, turn_on_logs};




#[test]
fn fail_types() {
    assert_eq!(fb("params/any_fail").build().is_err(), true);
}

#[test]
fn any_type() {
    turn_on_logs();
    struct X;
    impl Impl for X {
        fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
            let key = args.first().unwrap().clone().as_string().unwrap();
            let value = args.find_or_ith("arg".to_string(), 1).unwrap();
            println!("key: {:?}, value: {:?}", key, value);
            ctx.bb().lock().unwrap().put(key, value).unwrap();
            Ok(TickResult::success())
        }
    }

    let mut fb = fb("params/any");
    fb.register_sync_action("consumer", X);
    let mut f = fb.build().unwrap();
    let result = f.run();
    assert_eq!(result, Ok(TickResult::success()));

    let bb = f.bb.lock().unwrap();
    let a = bb.get("a".to_string()).unwrap().unwrap().clone().as_int().unwrap();
    let b = bb.get("b".to_string()).unwrap().unwrap().clone().as_string().unwrap();
    assert_eq!(a, 1);
    assert_eq!(b, "2".to_string());
}

