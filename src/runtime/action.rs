use std::collections::HashMap;
use crate::runtime::{RuntimeError, TickResult};
use crate::runtime::args::RtArgs;
use crate::runtime::rtree::TreeContext;

pub type ActionName = String;

#[derive(Default)]
pub struct ActionKeeper {
    actions: HashMap<ActionName, Box<dyn Impl>>,
    async_actions: HashMap<ActionName, Box<dyn AsyncImpl>>,
}

impl ActionKeeper {
    pub fn register_sync<T>(&mut self, name: ActionName, action: T) -> Result<(), RuntimeError>
        where T: Impl
    {
        &self.actions.insert(name,Box::new(action));
        Ok(())
    }

    pub fn register_async<T>(&mut self, name: ActionName, action: T) -> Result<(), RuntimeError>
        where T: AsyncImpl
    {
        &self.async_actions.insert(name,Box::new(action));
        Ok(())
    }
}


pub trait Impl {
    fn action_on_tick(&mut self, args: RtArgs, ctx: &mut TreeContext) -> Result<TickResult, RuntimeError>;
}

pub trait AsyncImpl {
    fn start(&mut self, args: RtArgs, ctx: &mut TreeContext) -> Result<TickResult, RuntimeError>;
    fn poll(&mut self, args: RtArgs, ctx: &mut TreeContext) -> Result<TickResult, RuntimeError>;
}



