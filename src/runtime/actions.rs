use crate::runtime::action::Impl;
use crate::runtime::args::RtArgs;
use crate::runtime::rtree::TreeContext;
use crate::runtime::{RuntimeErrorCause, TickResult};

struct Empty(String);

impl Impl for Empty {
    fn action_on_tick(
        &mut self,
        args: RtArgs,
        ctx: &mut TreeContext,
    ) -> Result<TickResult, RuntimeErrorCause> {
        Err(RuntimeErrorCause::UnImplementedAction(
            (&self).0.to_string(),
        ))
    }
}
