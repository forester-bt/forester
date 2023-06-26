use crate::runtime::action::builtin::Fail;
use crate::runtime::action::Action;
use crate::runtime::tests::fb;

#[test]
fn simple_inverter() {
    let mut fb = fb("decorators/simple_inv");

    fb.register_action("fail".to_string(), Action::sync(Fail));

    let mut f = fb.build().unwrap();
    let result = f.perform();
    println!("{:?}", result);
    assert_eq!(1, 0)
}
