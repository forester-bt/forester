use std::borrow::Cow;
use std::env;
use std::fmt::{Debug, Formatter};
use crate::runtime::action::{Action, ActionName, Impl, Tick};
use crate::runtime::action::builtin::ReturnResult;
use crate::runtime::action::keeper::ActionImpl;
use crate::runtime::args::RtArgs;
use crate::runtime::context::TreeContextRef;
use crate::runtime::{RtResult, TickResult};

pub(crate) fn action_impl(action: &ActionName) -> RtResult<Action> {
    Ok(Action::sync(ReturnResult::success()))
}

pub struct SinglePublisher;

impl Impl for SinglePublisher {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        Ok(TickResult::success())
    }
}

#[derive(Clone, Debug, Default)]
pub struct ForesterRosMessage {
    tp: String,
    msg: String,
}


