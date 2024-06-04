use std::future::Future;
use std::pin::Pin;
use std::sync::atomic::Ordering::Relaxed;
use tokio_util::sync::CancellationToken;
use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::TreeContextRef;
use crate::runtime::env::daemon::{AsyncDaemonFn, Daemon, DaemonFn, StopFlag};
use crate::runtime::env::daemon::context::DaemonContext;
use crate::runtime::{blackboard, TickResult};
use crate::tests::{fb, turn_on_logs};

struct Test;

impl DaemonFn for Test {
    fn perform(&mut self, ctx: DaemonContext, signal: StopFlag) {
        while !signal.load(Relaxed) {
            std::thread::sleep(std::time::Duration::from_millis(500));
            blackboard::utils::push_to_arr(ctx.bb.clone(), "test".into(), RtValue::int(1)).unwrap();
        }
    }
}

impl Impl for Test {
    fn tick(&self, _args: RtArgs, ctx: TreeContextRef) -> Tick {
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
    fb.register_sync_action("test", Test);
    fb.register_named_daemon("test_daemon".to_string(), Daemon::sync(Test));

    let mut forester = fb.build().unwrap();
    let result = forester.run_until(Some(10)).unwrap();
    assert_eq!(result, TickResult::success());

    let bb = forester.bb.lock().unwrap();
    let v = bb.get("test".to_string()).unwrap().unwrap();

    assert_eq!(v.clone().as_vec(|v| v.as_int().unwrap()).unwrap(), vec![1, 1, 1, 1]);
}


struct TestAction;

impl Impl for TestAction {
    fn tick(&self, _args: RtArgs, ctx: TreeContextRef) -> Tick {
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
        } else {
            Ok(TickResult::success())
        }
    }
}

struct DaemonSync;

impl DaemonFn for DaemonSync {
    fn perform(&mut self, ctx: DaemonContext, signal: StopFlag) {
        while !signal.load(Relaxed) {
            std::thread::sleep(std::time::Duration::from_millis(50));
            let mut bb = ctx.bb.lock().unwrap();
            let v = bb.get("test".to_string()).expect("no errors")
                .cloned().unwrap_or(RtValue::int(0));

            bb.put("test_daemon".to_string(), v).unwrap();
        }
    }
}

#[test]
fn built_in() {
    turn_on_logs();
    let mut fb = fb("daemons/builtin");
    fb.register_sync_action("test", TestAction);
    fb.register_named_daemon("daemon".to_string(), Daemon::sync(DaemonSync));

    let mut forester = fb.build().unwrap();
    let result = forester.run_until(Some(10)).unwrap();
    assert_eq!(result, TickResult::success());

    let bb = forester.bb.lock().unwrap();
    let v1 = bb.get("test".to_string()).unwrap().unwrap();
    let v2 = bb.get("test_daemon".to_string()).unwrap().unwrap();

    assert_eq!(v1.clone().as_int().unwrap(), 5);
    assert_eq!(v2.clone().as_int().unwrap(), 4);
}

impl AsyncDaemonFn for DaemonSync {
    fn prepare(&mut self, ctx: DaemonContext, signal: CancellationToken) -> Pin<Box<dyn Future<Output=()> + Send>> {
        Box::pin(async move {
            loop {
                tokio::select! {
                _ = signal.cancelled() => {
                    return;
                }
                _ = tokio::time::sleep(std::time::Duration::from_millis(10)) => {
                    let mut bb = ctx.bb.lock().unwrap();
                    let v = bb.get("test".to_string()).expect("no errors")
                        .cloned().unwrap_or(RtValue::int(0));

                    bb.put("test_daemon".to_string(), v).unwrap();
                }
            }
            }
        })
    }
}

#[test]
fn built_in_async() {
    let mut fb = fb("daemons/builtin");
    fb.register_sync_action("test", TestAction);
    fb.register_named_daemon("daemon".to_string(), Daemon::a_sync(DaemonSync));

    let mut forester = fb.build().unwrap();
    let result = forester.run_until(Some(10)).unwrap();
    assert_eq!(result, TickResult::success());

    let bb = forester.bb.lock().unwrap();
    let v1 = bb.get("test".to_string()).unwrap().unwrap();
    let v2 = bb.get("test_daemon".to_string()).unwrap().unwrap();

    assert_eq!(v1.clone().as_int().unwrap(), 5);
    let i = v2.clone().as_int().unwrap();
    assert!(i == 4 || i == 3);
}