use crate::runtime::tests::fb;
use crate::runtime::TickResult;

#[test]
fn builtin_actions() {
    let mut fb = fb("actions/builtin");

    let mut f = fb.build().unwrap();
    let result = f.start();
    assert_eq!(result, Ok(TickResult::failure("fail".to_string())));
}
