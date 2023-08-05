use crate::runtime::action::builtin::data::GenerateData;
use crate::runtime::action::builtin::ReturnResult;
use crate::runtime::action::{Action, Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::{TreeContext, TreeContextRef};
use crate::runtime::TickResult;
use crate::tests::{fb, test_folder, turn_on_logs};
use crate::tracer::{Tracer, TracerConfig};

#[test]
fn pointers() {
    let mut fb = fb("units/pointers");
    let mut forester = fb.build().unwrap();
    let result = forester.run().unwrap();

    assert_eq!(result, TickResult::success())
}
