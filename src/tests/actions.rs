use crate::runtime::args::RtValue;
use crate::runtime::TickResult;
use crate::tests::{fb, test_folder, turn_on_logs};
use crate::tracer::{Tracer, TracerConfig};

#[test]
fn builtin_actions() {
    let mut fb = fb("actions/builtin");

    let mut f = fb.build().unwrap();
    let result = f.run();
    assert_eq!(result, Ok(TickResult::failure("test".to_string())));
}

#[test]
fn lock_unlock() {
    let mut fb = fb("actions/lock_unlock");

    let mut f = fb.build().unwrap();
    let result = f.run();
    assert_eq!(result, Ok(TickResult::success()));
    let guard = f.bb.lock().unwrap();
    let k = guard.get("k".to_string()).unwrap();
    assert_eq!(k, Some(&RtValue::str("v2".to_string())))
}

#[test]
fn http_get() {
    let mut fb = fb("actions/simple_http");
    let mut f = fb.build().unwrap();
    let result = f.run();
    let bb_ref = f.bb.lock().unwrap();
    let out1 = bb_ref
        .get("out1".to_string())
        .unwrap()
        .and_then(|v| v.clone().as_string())
        .unwrap();
    let out2 = bb_ref
        .get("out2".to_string())
        .unwrap()
        .and_then(|v| v.clone().as_string())
        .unwrap();

    assert_eq!(result, Ok(TickResult::success()));
    assert!(out1.contains("https://google.com"));
    assert!(out2.contains("https://google.com"));
}
