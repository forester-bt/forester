use crate::runtime::action::builtin::{Fail, GenerateData};
use crate::runtime::action::Action;
use crate::runtime::args::RtValue;
use crate::runtime::tests::fb;
use crate::runtime::TickResult;

#[test]
fn simple_inverter() {
    let mut fb = fb("decorators/simple_inv");

    fb.register_action(
        "store".to_string(),
        Action::sync(GenerateData::new(
            "key".to_string(),
            RtValue::String("data".to_string()),
            |v| v,
        )),
    );

    let mut f = fb.build().unwrap();
    let result = f.perform();
    assert_eq!(
        result,
        Ok(TickResult::failure(format!(
            "decorator inverts the result."
        )))
    );
    let x =
        f.bb.get("key".to_string())
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

    fb.register_action(
        "store".to_string(),
        Action::sync(GenerateData::new(
            "key".to_string(),
            RtValue::String("1".to_string()),
            |v| {
                let curr = v.as_string().unwrap();
                RtValue::String(format!("{curr}1"))
            },
        )),
    );

    let mut f = fb.build().unwrap();
    let result = f.perform();
    assert_eq!(result, Ok(TickResult::success()));

    let x =
        f.bb.get("key".to_string())
            .unwrap()
            .unwrap()
            .clone()
            .as_string()
            .unwrap();
    assert_eq!(x.as_str(), "11111")
}
