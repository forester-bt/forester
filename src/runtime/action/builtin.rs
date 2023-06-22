use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::RtArgs;
use crate::runtime::rtree::TreeContext;
use crate::runtime::{RuntimeErrorCause, TickResult};

struct Empty(String);

impl Impl for Empty {
    fn tick(&self, _args: RtArgs, _ctx: &mut TreeContext) -> Tick {
        Err(RuntimeErrorCause::UnImplementedAction(self.0.to_string()))
    }
}
