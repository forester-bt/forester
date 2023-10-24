use std::future::Future;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering::Relaxed;
use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::TreeContextRef;
use crate::runtime::env::daemon::{Daemon, DaemonContext};
use crate::runtime::{blackboard, RtOk, TickResult};
use crate::tests::{fb, turn_on_logs};

struct Test(Arc<AtomicBool>);

impl Daemon for Test {
    fn perform(&mut self, ctx: DaemonContext) {
        while !self.signal().load(Relaxed) {
            std::thread::sleep(std::time::Duration::from_millis(500));
            blackboard::utils::push_to_arr(ctx.bb.clone(), "test".into(), RtValue::int(1)).unwrap();
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


struct TestAction;

impl Impl for TestAction {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        std::thread::sleep(std::time::Duration::from_millis(100));
        let arc = ctx.bb();
        let mut bb = arc.lock()?;
        let val =
            bb.get("test".to_string())
                .expect("no errors")
                .cloned().and_then(|v| v.as_int())
                .unwrap_or(0);

        bb.put("test".to_string(), RtValue::int(val + 1))?;

        let t = ctx.current_tick();
        if t < 4 {
            Ok(TickResult::running())
        }  else {
            Ok(TickResult::success())
        }
    }
}

struct DaemonSync(Arc<AtomicBool>);

impl Daemon for DaemonSync {
    fn perform(&mut self, ctx: DaemonContext) {
        while !self.signal().load(Relaxed) {
            std::thread::sleep(std::time::Duration::from_millis(50));
            let mut bb = ctx.bb.lock().unwrap();
            let v = bb.get("test".to_string()).expect("no errors")
                .cloned().unwrap_or(RtValue::int(0));

            bb.put("test_daemon".to_string(), v).unwrap();
        }
    }

    fn signal(&self) -> Arc<AtomicBool> {
        self.0.clone()
    }
}

#[test]
fn built_in() {
    turn_on_logs();
    let mut fb = fb("daemons/builtin");
    let signal = Arc::new(AtomicBool::new(false));
    fb.register_sync_action("test", TestAction);
    fb.register_named_daemon("daemon".to_string(), DaemonSync(signal));

    let mut forester = fb.build().unwrap();
    let result = forester.run_until(Some(10)).unwrap();
    assert_eq!(result, TickResult::success());

    let bb = forester.bb.lock().unwrap();
    let v1 = bb.get("test".to_string()).unwrap().unwrap();
    let v2 = bb.get("test_daemon".to_string()).unwrap().unwrap();

    assert_eq!(v1.clone().as_int().unwrap(), 5);
    assert_eq!(v2.clone().as_int().unwrap(), 4);
}

