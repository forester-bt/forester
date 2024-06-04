use crate::runtime::action::builtin::data::{GenerateData, StoreData};


use crate::runtime::args::RtValue;
use crate::runtime::TickResult;
use crate::tests::{fb, turn_on_logs};
use std::time::SystemTime;

#[test]
fn simple_inverter() {
    let mut fb = fb("decorators/simple_inv");

    fb.register_sync_action("store", StoreData);

    let mut f = fb.build().unwrap();
    let result = f.run();
    assert_eq!(
        result,
        Ok(TickResult::failure(format!(
            "decorator inverts the result."
        )))
    );
    let x =
        f.bb.lock()
            .unwrap()
            .get("key".to_string())
            .unwrap()
            .unwrap()
            .clone()
            .as_string()
            .unwrap();
    assert_eq!(x.as_str(), "data")
}

#[test]
fn simple_repeat() {
    let mut fb = fb("decorators/simple_repeat");

    fb.register_sync_action(
        "store",
        GenerateData::new(|v| {
            let curr = v.as_string().unwrap();
            RtValue::String(format!("{curr}1"))
        }),
    );

    let mut f = fb.build().unwrap();

    let result = f.run();
    assert_eq!(result, Ok(TickResult::success()));

    let x =
        f.bb.lock()
            .unwrap()
            .get("key".to_string())
            .unwrap()
            .unwrap()
            .clone()
            .as_string()
            .unwrap();
    assert_eq!(x.as_str(), "111111")
}
#[test]
fn simple_retry() {
    let mut fb = fb("decorators/simple_retry");

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
    assert_eq!(x, 5)
}
#[test]
fn simple_delay() {
    let mut fb = fb("decorators/simple_delay");

    fb.register_sync_action("store", StoreData);

    let before = SystemTime::now();

    let mut f = fb.build().unwrap();
    let result = f.run();
    assert_eq!(result, Ok(TickResult::success()));

    let duration = SystemTime::now().duration_since(before).unwrap();
    assert!(duration.as_micros() >= 2000);

    let x =
        f.bb.lock()
            .unwrap()
            .get("key".to_string())
            .unwrap()
            .unwrap()
            .clone()
            .as_string()
            .unwrap();
    assert_eq!(x.as_str(), "1")
}
#[test]
fn repeat_reactive() {
    turn_on_logs();
    let mut fb = fb("decorators/repeat_reactive");

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
    assert_eq!(x, 5)
}
#[test]
fn repeat_failure() {
    let fb = fb("decorators/repeat_failure");

    let mut f = fb.build().unwrap();
    assert_eq!(f.run(), Ok(TickResult::failure("test".to_string())));

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
    assert_eq!(x, 1)
}
#[test]
fn repeat_repeat() {
    let mut fb = fb("decorators/repeat_repeat");

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
    assert_eq!(x, 12)
}
#[test]
fn retry_retry() {
    let mut fb = fb("decorators/retry_retry");

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
    assert_eq!(x, 15)
}
