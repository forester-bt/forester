use crate::runtime::args::RtValue;
use crate::runtime::TickResult;
use crate::tests::fb;

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
fn mix_test() {
    let fb = fb("actions/mix_test");
    let mut f = fb.build().unwrap();
    assert_eq!(f.run(), Ok(TickResult::success()));
}
