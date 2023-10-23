use std::future::Future;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering::Relaxed;
use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::blackboard::BBHelper;
use crate::runtime::context::TreeContextRef;
use crate::runtime::env::daemon::{Daemon, DaemonContext};
use crate::runtime::env::RtEnv;
use crate::runtime::{RtOk, TickResult};
use crate::tests::{fb, turn_on_logs};
use crate::tree::parser::ast::message::Bool;

struct Test(Arc<AtomicBool>);

impl Daemon for Test {
    fn start(&mut self, ctx: DaemonContext) {
        while !self.signal().load(Relaxed) {
            std::thread::sleep(std::time::Duration::from_millis(500));
            BBHelper::push_to_arr(ctx.bb.clone(), "test".into(), RtValue::int(1)).unwrap();
        }
    }

    fn signal(&self) -> Arc<AtomicBool> {
        self.0.clone()
    }
}

impl Impl for Test {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        std::thread::sleep(std::time::Duration::from_millis(500));
        if ctx.current_tick() < 4 {
            Ok(TickResult::running())
        } else {
            Ok(TickResult::success())
        }
    }
}

#[test]
fn smoke() {
    turn_on_logs();
    let mut fb = fb("daemons/smoke");
    let signal = Arc::new(AtomicBool::new(false));
    fb.register_sync_action("test", Test(signal.clone()));
    fb.register_named_daemon("test_daemon".to_string(), Test(signal));

    let mut forester = fb.build().unwrap();
    let result = forester.run_until(Some(10)).unwrap();
    assert_eq!(result, TickResult::success());

    let bb = forester.bb.lock().unwrap();
    let v = bb.get("test".to_string()).unwrap().unwrap();

    assert_eq!(v.clone().as_vec(|v| v.as_int().unwrap()).unwrap(), vec![1, 1, 1, 1]);
}

