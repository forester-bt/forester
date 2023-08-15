use crate::runtime::action::builtin::data::GenerateData;
use crate::runtime::action::builtin::ReturnResult;
use crate::runtime::action::{Action, Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::{TreeContext, TreeContextRef};
use crate::runtime::TickResult;
use crate::tests::{fb, test_folder, turn_on_logs};

struct StoreTick;

impl Impl for StoreTick {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
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

    let buf = test_folder("flow/sequence/bb_dump.json");
}

#[test]
fn seq_restart_all_children() {
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

    f.bb.lock().unwrap().print_dump();
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
