use crate::runtime::TickResult;
use crate::tests::{fb, test_folder, turn_on_logs};
use crate::tracer::{Tracer, TracerConfiguration};

#[test]
fn builtin_actions() {
    turn_on_logs();
    let mut fb = fb("actions/builtin");

    let mut f = fb.build().unwrap();
    let result = f.run();
    assert_eq!(result, Ok(TickResult::failure("test".to_string())));
}
#[test]
fn http_get() {
    turn_on_logs();
    let mut fb = fb("actions/simple_http");
    let mut f = fb.build().unwrap();
    let result = f.run();
    f.bb.print_dump().unwrap();
    assert_eq!(result, Ok(TickResult::success()));
}
#[test]
fn http_get_async() {
    turn_on_logs();
    let mut fb = fb("actions/simple_http_async");
    fb.tracer(
        Tracer::create(TracerConfiguration::in_file(test_folder(
            "actions/simple_http_async/trace.log",
        )))
        .unwrap(),
    );
    let mut f = fb.build().unwrap();
    let result = f.run();
    f.bb.print_dump().unwrap();
    assert_eq!(result, Ok(TickResult::success()));
}
