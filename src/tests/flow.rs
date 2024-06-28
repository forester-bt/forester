use crate::runtime::action::builtin::data::GenerateData;

use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::TreeContextRef;
use crate::runtime::TickResult;
use crate::tests::{fb, test_folder, turn_on_logs};
struct StoreTick;

impl Impl for StoreTick {
    fn tick(&self, _args: RtArgs, ctx: TreeContextRef) -> Tick {
        let ts = ctx.current_tick();
        ctx.bb()
            .lock()
            .unwrap()
            .put("tick".to_string(), RtValue::int(ts as i64))?;

        Ok(TickResult::Success)
    }
}

#[test]
fn simple_sequence() {
    let mut fb = fb("flow/sequence");

    fb.register_sync_action("store", GenerateData::new(|v| v));
    fb.register_sync_action("store_tick", StoreTick);

    let mut f = fb.build().unwrap();
    let result = f.run();
    assert_eq!(result, Ok(TickResult::success()));

    let x =
        f.bb.lock()
            .unwrap()
            .get("a".to_string())
            .ok()
            .flatten()
            .and_then(|v| v.clone().as_string())
            .unwrap();
    assert_eq!(x.as_str(), "1");

    let x =
        f.bb.lock()
            .unwrap()
            .get("b".to_string())
            .ok()
            .flatten()
            .and_then(|v| v.clone().as_string())
            .unwrap();
    assert_eq!(x.as_str(), "2");

    let x =
        f.bb.lock()
            .unwrap()
            .get("c".to_string())
            .ok()
            .flatten()
            .and_then(|v| v.clone().as_string())
            .unwrap();
    assert_eq!(x.as_str(), "3");

    let x =
        f.bb.lock()
            .unwrap()
            .get("tick".to_string())
            .ok()
            .flatten()
            .and_then(|v| v.clone().as_int())
            .unwrap();
    assert_eq!(x, 1);

    let _buf = test_folder("flow/sequence/bb_dump.json");
}

#[test]
fn sequence_restart_all_children() {
    let mut fb = fb("flow/sequence_restart_children");

    fb.register_sync_action(
        "gen_store",
        GenerateData::new(|v| {
            let curr = v.as_int().unwrap_or(0);
            RtValue::int(curr + 1)
        }),
    );
    fb.register_sync_action("store_tick", StoreTick);

    let mut f = fb.build().unwrap();
    let result = f.run();
    assert_eq!(result, Ok(TickResult::failure("".to_string())));

    let x =
        f.bb.lock()
            .unwrap()
            .get("k1".to_string())
            .ok()
            .flatten()
            .and_then(|v| v.clone().as_int())
            .unwrap();
    assert_eq!(x, 5);

    let x =
        f.bb.lock()
            .unwrap()
            .get("k2".to_string())
            .ok()
            .flatten()
            .and_then(|v| v.clone().as_int())
            .unwrap();
    assert_eq!(x, 5);

    let x =
        f.bb.lock()
            .unwrap()
            .get("tick".to_string())
            .ok()
            .flatten()
            .and_then(|v| v.clone().as_int())
            .unwrap();
    assert_eq!(x, 5);
}

#[test]
fn mseq_restart_all_children() {
    let mut fb = fb("flow/msequence_restart_children");

    fb.register_sync_action(
        "gen_store",
        GenerateData::new(|v| {
            let curr = v.as_int().unwrap_or(0);
            RtValue::int(curr + 1)
        }),
    );
    fb.register_sync_action("store_tick", StoreTick);

    let mut f = fb.build().unwrap();
    let result = f.run();
    assert_eq!(result, Ok(TickResult::failure("".to_string())));

    let x =
        f.bb.lock()
            .unwrap()
            .get("k1".to_string())
            .ok()
            .flatten()
            .and_then(|v| v.clone().as_int())
            .unwrap();
    assert_eq!(x, 1);

    let x =
        f.bb.lock()
            .unwrap()
            .get("k2".to_string())
            .ok()
            .flatten()
            .and_then(|v| v.clone().as_int())
            .unwrap();
    assert_eq!(x, 1);

    let x =
        f.bb.lock()
            .unwrap()
            .get("tick".to_string())
            .ok()
            .flatten()
            .and_then(|v| v.clone().as_int())
            .unwrap();
    assert_eq!(x, 1);
}

#[test]
fn sequence_running() {
    turn_on_logs();
    let mut fb = fb("flow/sequence_running");

    fb.register_sync_action(
        "incr",
        GenerateData::new(|v| {
            let curr = v.as_int().unwrap_or(0);
            RtValue::int(curr + 1)
        }),
    );

    let mut f = fb.build().unwrap();
    let result = f.run_until(Some(11));

    f.bb.lock().unwrap().print_dump().unwrap();
    assert_eq!(result, Ok(TickResult::success()));

    let x =
        f.bb.lock()
            .unwrap()
            .get("a".to_string())
            .ok()
            .flatten()
            .and_then(|v| v.clone().as_int())
            .unwrap();
    assert_eq!(x, 2);
}

#[test]
fn sequence_reset_after_running_success() {
    // See comment in the tree file for what this is testing.
    let mut fb = fb("flow/sequence_reset_after_running_success");

    fb.register_sync_action(
        "incr",
        GenerateData::new(|v| RtValue::int(v.as_int().unwrap_or(0) + 1)),
    );

    let mut f = fb.build().unwrap();
    assert_eq!(f.run(), Ok(TickResult::success()));

    let x =
        f.bb.lock()
            .unwrap()
            .get("tick".to_string())
            .ok()
            .flatten()
            .unwrap()
            .clone()
            .as_int()
            .unwrap();
    assert_eq!(x, 8)
}

#[test]
fn sequence_reset_after_running_failure() {
    // See comment in the tree file for what this is testing.
    let mut fb = fb("flow/sequence_reset_after_running_failure");

    fb.register_sync_action(
        "incr",
        GenerateData::new(|v| RtValue::int(v.as_int().unwrap_or(0) + 1)),
    );

    let mut f = fb.build().unwrap();
    assert_eq!(
        f.run(),
        Ok(TickResult::failure(
            "decorator inverts the result.".to_string()
        ))
    );

    let x =
        f.bb.lock()
            .unwrap()
            .get("tick".to_string())
            .ok()
            .flatten()
            .unwrap()
            .clone()
            .as_int()
            .unwrap();
    assert_eq!(x, 10)
}

#[test]
fn sequence_handles_halt() {
    // See comment in the tree file for what this is testing.
    let mut fb = fb("flow/sequence_handles_halt");

    fb.tracer(crate::tracer::Tracer::default());
    fb.register_sync_action(
        "incr",
        GenerateData::new(|v| RtValue::int(v.as_int().unwrap_or(0) + 1)),
    );

    let mut f = fb.build().unwrap();
    assert_eq!(f.run(), Ok(TickResult::success()));

    println!("{}", f.tracer.lock().unwrap().to_string());
    let x =
        f.bb.lock()
            .unwrap()
            .get("x".to_string())
            .ok()
            .flatten()
            .unwrap()
            .clone()
            .as_int()
            .unwrap();
    assert_eq!(x, 2);

    let y =
        f.bb.lock()
            .unwrap()
            .get("y".to_string())
            .ok()
            .flatten()
            .unwrap()
            .clone()
            .as_int()
            .unwrap();
    assert_eq!(y, 7);
}

#[test]
fn r_sequence_halt_on_interrupt() {
    // See comment in the tree file for what this is testing.
    let mut fb = fb("flow/r_sequence_halt");

    fb.register_sync_action(
        "incr",
        GenerateData::new(|v| RtValue::int(v.as_int().unwrap_or(0) + 1)),
    );

    let mut f = fb.build().unwrap();
    assert_eq!(f.run(), Ok(TickResult::success()));

    let x =
        f.bb.lock()
            .unwrap()
            .get("x".to_string())
            .ok()
            .flatten()
            .unwrap()
            .clone()
            .as_int()
            .unwrap();
    assert_eq!(x, 7)
}

#[test]
fn fallback() {
    let mut fb = fb("flow/fallback");

    fb.register_sync_action("tick_num_store", StoreTick);

    let mut f = fb.build().unwrap();
    let result = f.run();
    assert_eq!(result, Ok(TickResult::success()));

    let x =
        f.bb.lock()
            .unwrap()
            .get("tick".to_string())
            .ok()
            .flatten()
            .and_then(|v| v.clone().as_int())
            .unwrap();
    assert_eq!(x, 1);
}

#[test]
fn fallback_retry() {
    turn_on_logs();

    let mut fb = fb("flow/fallback_retry");

    fb.register_sync_action("tick_num_store", StoreTick);

    let mut f = fb.build().unwrap();
    let result = f.run();
    assert_eq!(result, Ok(TickResult::success()));
}

#[test]
fn fallback_reset_after_running_failure() {
    // See comment in the tree file for what this is testing.
    let mut fb = fb("flow/fallback_reset_after_running_failure");

    fb.register_sync_action(
        "incr",
        GenerateData::new(|v| RtValue::int(v.as_int().unwrap_or(0) + 1)),
    );

    let mut f = fb.build().unwrap();
    assert_eq!(
        f.run(),
        Ok(TickResult::failure(
            "decorator inverts the result.".to_string()
        ))
    );

    let x =
        f.bb.lock()
            .unwrap()
            .get("tick".to_string())
            .ok()
            .flatten()
            .unwrap()
            .clone()
            .as_int()
            .unwrap();
    assert_eq!(x, 8)
}

#[test]
fn fallback_reset_after_running_success() {
    // See comment in the tree file for what this is testing.
    let mut fb = fb("flow/fallback_reset_after_running_success");

    fb.register_sync_action(
        "incr",
        GenerateData::new(|v| RtValue::int(v.as_int().unwrap_or(0) + 1)),
    );

    let mut f = fb.build().unwrap();
    assert_eq!(f.run(), Ok(TickResult::Success));

    let x =
        f.bb.lock()
            .unwrap()
            .get("tick".to_string())
            .ok()
            .flatten()
            .unwrap()
            .clone()
            .as_int()
            .unwrap();
    assert_eq!(x, 10)
}

#[test]
fn fallback_handles_halt() {
    // See comment in the tree file for what this is testing.
    let mut fb = fb("flow/fallback_handles_halt");

    fb.tracer(crate::tracer::Tracer::default());
    fb.register_sync_action(
        "incr",
        GenerateData::new(|v| RtValue::int(v.as_int().unwrap_or(0) + 1)),
    );

    let mut f = fb.build().unwrap();
    assert_eq!(
        f.run(),
        Ok(TickResult::failure(
            "decorator inverts the result.".to_string()
        ))
    );

    println!("{}", f.tracer.lock().unwrap().to_string());
    let x =
        f.bb.lock()
            .unwrap()
            .get("x".to_string())
            .ok()
            .flatten()
            .unwrap()
            .clone()
            .as_int()
            .unwrap();
    assert_eq!(x, 2);

    let y =
        f.bb.lock()
            .unwrap()
            .get("y".to_string())
            .ok()
            .flatten()
            .unwrap()
            .clone()
            .as_int()
            .unwrap();
    assert_eq!(y, 7);
}

#[test]
fn r_fallback_halt_on_interrupt() {
    // See comment in the tree file for what this is testing.
    let mut fb = fb("flow/r_fallback_halt");

    fb.register_sync_action(
        "incr",
        GenerateData::new(|v| RtValue::int(v.as_int().unwrap_or(0) + 1)),
    );

    let mut f = fb.build().unwrap();
    assert_eq!(
        f.run(),
        Ok(TickResult::failure(
            "decorator inverts the result.".to_string()
        ))
    );

    let x =
        f.bb.lock()
            .unwrap()
            .get("x".to_string())
            .ok()
            .flatten()
            .unwrap()
            .clone()
            .as_int()
            .unwrap();
    assert_eq!(x, 7)
}

#[test]
fn parallel_simple() {
    turn_on_logs();

    struct Condition;
    impl Impl for Condition {
        fn tick(&self, _args: RtArgs, ctx: TreeContextRef) -> Tick {
            if ctx.current_tick() > 2 {
                Ok(TickResult::Success)
            } else {
                Ok(TickResult::Failure("".to_string()))
            }
        }
    }

    let mut fb = fb("flow/parallel/simple");
    fb.register_sync_action("fail_before_tick", Condition);

    let mut f = fb.build().unwrap();
    let result = f.run_until(Some(5));

    let bb = f.bb.lock().unwrap();
    assert_eq!(result, Ok(TickResult::success()));

    let t1 = bb
        .get("tick1".to_string())
        .ok()
        .flatten()
        .unwrap()
        .clone()
        .as_int();
    assert_eq!(t1, Some(3));
    let t2 = bb
        .get("tick2".to_string())
        .ok()
        .flatten()
        .unwrap()
        .clone()
        .as_int();
    assert_eq!(t2, Some(1));
}
#[test]
fn parallel_simple_w_retry() {
    turn_on_logs();

    struct Condition;
    impl Impl for Condition {
        fn tick(&self, _args: RtArgs, ctx: TreeContextRef) -> Tick {
            if ctx.current_tick() > 2 {
                Ok(TickResult::Success)
            } else {
                Ok(TickResult::Failure("".to_string()))
            }
        }
    }

    let mut fb = fb("flow/parallel/simple_w_retry");

    fb.register_sync_action("fail_before_tick", Condition);
    fb.register_sync_action(
        "incr",
        GenerateData::new(|v| RtValue::int(v.as_int().unwrap_or(0) + 1)),
    );

    let mut f = fb.build().unwrap();
    let result = f.run_until(Some(20));

    let bb = f.bb.lock().unwrap();
    assert_eq!(result, Ok(TickResult::success()));

    let t1 = bb
        .get("t1".to_string())
        .ok()
        .flatten()
        .unwrap()
        .clone()
        .as_int();
    assert_eq!(t1, Some(3));
    let t2 = bb
        .get("t2".to_string())
        .ok()
        .flatten()
        .unwrap()
        .clone()
        .as_int();
    assert_eq!(t2, Some(3));
}
