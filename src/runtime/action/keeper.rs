use crate::runtime::action::{Action, ActionName};
use crate::runtime::{RtResult, RuntimeErrorCause};
use std::collections::HashMap;

#[derive(Default)]
pub struct ActionKeeper {
    actions: HashMap<ActionName, Action>,
}

impl ActionKeeper {
    pub fn register(&mut self, name: ActionName, action: Action) -> RtResult<()> {
        &self.actions.insert(name, action);
        Ok(())
    }
}
