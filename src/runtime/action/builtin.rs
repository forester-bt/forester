pub mod data;

use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::blackboard::{BBKey, BlackBoard};
use crate::runtime::context::TreeContext;
use crate::runtime::{RuntimeError, TickResult};

pub struct Fail;

impl Impl for Fail {
    fn tick(&self, args: RtArgs, _ctx: &mut TreeContext) -> Tick {
        let c = args
            .first_as(RtValue::as_string)
            .unwrap_or("fail".to_string());
        Ok(TickResult::failure(c))
    }
}
