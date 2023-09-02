use crate::runtime::action::builtin::remote::RemoteHttpAction;
use crate::runtime::args::RtValue;
use crate::runtime::env::RtEnv;
use crate::runtime::TickResult;
use crate::tests::fb;
use serde_json::json;
use wiremock::matchers::{method, path};
use wiremock::{Mock, MockServer, ResponseTemplate};

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
