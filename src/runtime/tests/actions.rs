use crate::runtime::tests::fb;
use crate::runtime::TickResult;
use crate::turn_on_logs;

#[test]
fn builtin_actions() {
    turn_on_logs();
    let mut fb = fb("actions/builtin");

    let mut f = fb.build().unwrap();
    let result = f.start();
    assert_eq!(result, Ok(TickResult::failure("test".to_string())));
}
