use crate::runtime::action::builtin::data::{GenerateData, StoreData};
use crate::runtime::action::builtin::ReturnResult;
use crate::runtime::action::Action;
use crate::runtime::args::RtValue;
use crate::runtime::TickResult;
use crate::tests::fb;
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
