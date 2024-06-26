use std::sync::{Arc, Mutex};

use crate::runtime::action::Impl;
use crate::runtime::args::RtValue;
use crate::runtime::TickResult;
use crate::tests::fb;

struct HaltTester {
    pub err_on_halt: bool,
    pub halt_called: Arc<Mutex<i32>>,
}

impl Impl for HaltTester {
    fn tick(
        &self,
        args: crate::runtime::args::RtArgs,
        ctx: crate::runtime::context::TreeContextRef,
    ) -> crate::runtime::action::Tick {
        let _ = args;
        let _ = ctx;
        Ok(TickResult::running())
    }

    fn halt(
        &self,
        args: crate::runtime::args::RtArgs,
        ctx: crate::runtime::context::TreeContextRef,
    ) -> crate::runtime::RtOk {
        let _ = args;
        let _ = ctx;
        let mut halt_called = self.halt_called.lock().unwrap();
        *halt_called += 1;
        if self.err_on_halt {
            return Err(crate::runtime::RuntimeError::UnImplementedAction(
                "testing halt errors properly".to_string(),
            ));
        } else {
            Ok(())
        }
    }
}

#[test]
fn builtin_actions() {
    let fb = fb("actions/builtin");

    let mut f = fb.build().unwrap();
    let result = f.run();
    assert_eq!(result, Ok(TickResult::failure("test".to_string())));
}

#[test]
fn lock_unlock() {
    let fb = fb("actions/lock_unlock");

    let mut f = fb.build().unwrap();
    let result = f.run();
    assert_eq!(result, Ok(TickResult::success()));
    let guard = f.bb.lock().unwrap();
    let k = guard.get("k".to_string()).unwrap();
    assert_eq!(k, Some(&RtValue::str("v2".to_string())))
}

#[test]
fn builtin_http_get() {
    let fb = fb("actions/simple_http");
    let mut f = fb.build().unwrap();
    assert_eq!(f.run(), Ok(TickResult::success()));
}
#[test]
fn builtin_test() {
    let fb = fb("actions/builtin_test");
    let mut f = fb.build().unwrap();
    assert_eq!(f.run(), Ok(TickResult::success()));
}

#[test]
#[ignore]
fn mix_test() {
    let fb = fb("actions/mix_test");
    let mut f = fb.build().unwrap();
    assert_eq!(f.run(), Ok(TickResult::success()));
}

#[test]
fn sync_action_halt_ok() {
    let mut fb = fb("actions/sync_halt");

    // Set up halt tester so we can query it later
    let halt_called = Arc::new(Mutex::new(0));
    let halt_tester = HaltTester {
        err_on_halt: false,
        halt_called: halt_called.clone(),
    };

    fb.tracer(crate::tracer::Tracer::default());
    fb.register_sync_action("halt_tester", halt_tester);

    let mut f = fb.build().unwrap();
    assert_eq!(f.run(), Ok(TickResult::success()));

    println!("{}", f.tracer.lock().unwrap().to_string());

    // Check that halt was called exactly once
    assert_eq!(*halt_called.lock().unwrap(), 1);
}

#[test]
fn sync_action_halt_err() {
    let mut fb = fb("actions/sync_halt");

    // Set up halt tester so we can query it later
    let halt_called = Arc::new(Mutex::new(0));
    let halt_tester = HaltTester {
        err_on_halt: true,
        halt_called: halt_called.clone(),
    };

    fb.tracer(crate::tracer::Tracer::default());
    fb.register_sync_action("halt_tester", halt_tester);

    let mut f = fb.build().unwrap();
    let result = f.run();

    println!("{}", f.tracer.lock().unwrap().to_string());
    assert_eq!(
        result,
        Err(crate::runtime::RuntimeError::UnImplementedAction(
            "testing halt errors properly".to_string(),
        ))
    );

    // Check that halt was called exactly once
    assert_eq!(*halt_called.lock().unwrap(), 1);
}
