use crate::runtime::args::RtArgs;
use crate::runtime::rtree::TreeContext;
use crate::runtime::{RuntimeErrorCause, TickResult};
use std::collections::HashMap;

pub type ActionName = String;

#[derive(Default)]
pub struct ActionKeeper {
    actions: HashMap<ActionName, Box<dyn Impl>>,
    async_actions: HashMap<ActionName, Box<dyn AsyncImpl>>,
}

impl ActionKeeper {
    pub fn register_sync<T>(&mut self, name: ActionName, action: T) -> Result<(), RuntimeErrorCause>
    where
        T: Impl + 'static,
    {
        &self.actions.insert(name, Box::new(action));
        Ok(())
    }

    pub fn register_async<T>(
        &mut self,
        name: ActionName,
        action: T,
    ) -> Result<(), RuntimeErrorCause>
    where
        T: AsyncImpl + 'static,
    {
        &self.async_actions.insert(name, Box::new(action));
        Ok(())
    }
}

pub trait Impl {
    fn action_on_tick(
        &mut self,
        args: RtArgs,
        ctx: &mut TreeContext,
    ) -> Result<TickResult, RuntimeErrorCause>;
}

pub trait AsyncImpl {
    fn start(
        &mut self,
        args: RtArgs,
        ctx: &mut TreeContext,
    ) -> Result<TickResult, RuntimeErrorCause>;
    fn poll(
        &mut self,
        args: RtArgs,
        ctx: &mut TreeContext,
    ) -> Result<TickResult, RuntimeErrorCause>;
}
