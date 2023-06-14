use crate::runtime::args::RtArgs;
use crate::runtime::rtree::TreeContext;
use crate::runtime::{RuntimeError, TickResult};
use crate::runtime::action::Impl;

struct Empty(String);


impl Impl for Empty {
    fn action_on_tick(&mut self, args: RtArgs, ctx: &mut TreeContext) -> Result<TickResult, RuntimeError> {
        Err(RuntimeError::UnImplementedAction((&self).0.to_string()))
    }
}