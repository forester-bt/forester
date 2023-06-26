pub mod builtin;
pub mod decorator;
pub mod flow;
pub mod keeper;

use crate::runtime::args::RtArgs;
use crate::runtime::context::TreeContext;
use crate::runtime::{RtResult, RuntimeError, TickResult};
use std::collections::HashMap;

pub type ActionName = String;
pub type Tick = RtResult<TickResult>;

pub enum Action {
    Impl(Box<dyn Impl>),
    Async(Box<dyn ImplAsync>),
}

impl Action {
    pub fn sync<T>(a: T) -> Self
    where
        T: Impl + 'static,
    {
        Action::Impl(Box::new(a))
    }
}

impl Action {
    pub fn tick(&self, args: RtArgs, ctx: &mut TreeContext) -> Tick {
        match self {
            Action::Impl(a) => a.tick(args, ctx),
            Action::Async(aa) => aa.tick(args, ctx),
        }
    }
}

pub trait Impl {
    fn tick(&self, args: RtArgs, ctx: &mut TreeContext) -> Tick;
}

pub trait ImplAsync {
    fn tick(&self, args: RtArgs, ctx: &mut TreeContext) -> Tick;
    fn halt(&self, ctx: &mut TreeContext) -> Tick;
}

impl From<Box<dyn Impl>> for Action {
    fn from(value: Box<dyn Impl>) -> Self {
        Action::Impl(value)
    }
}

impl From<Box<dyn ImplAsync>> for Action {
    fn from(value: Box<dyn ImplAsync>) -> Self {
        Action::Async(value)
    }
}
