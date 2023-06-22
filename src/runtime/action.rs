pub mod builtin;
pub mod keeper;

use crate::runtime::args::RtArgs;
use crate::runtime::rtree::TreeContext;
use crate::runtime::{RtResult, RuntimeErrorCause, TickResult};
use std::collections::HashMap;

pub type ActionName = String;
pub type Tick = RtResult<TickResult>;

pub enum Action {
    Impl(Box<dyn Impl>),
    Mut(Box<dyn ImplMut>),
    Async(Box<dyn ImplAsync>),
}

pub trait ImplMut {
    fn tick(&mut self, args: RtArgs, ctx: &mut TreeContext) -> Tick;
}

pub trait Impl {
    fn tick(&self, args: RtArgs, ctx: &mut TreeContext) -> Tick;
}

pub trait ImplAsync {
    fn tick(&mut self, args: RtArgs, ctx: &mut TreeContext) -> Tick;
    fn poll(&mut self, ctx: &mut TreeContext) -> Tick;
    fn halt(&mut self, ctx: &mut TreeContext) -> Tick;
}

impl From<Box<dyn Impl>> for Action {
    fn from(value: Box<dyn Impl>) -> Self {
        Action::Impl(value)
    }
}
impl From<Box<dyn ImplMut>> for Action {
    fn from(value: Box<dyn ImplMut>) -> Self {
        Action::Mut(value)
    }
}
impl From<Box<dyn ImplAsync>> for Action {
    fn from(value: Box<dyn ImplAsync>) -> Self {
        Action::Async(value)
    }
}
