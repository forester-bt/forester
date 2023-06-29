use crate::runtime::action::{Action, ActionName};
use crate::runtime::args::RtArgs;
use crate::runtime::context::{RNodeState, TreeContext};
use crate::runtime::{RtResult, RuntimeError};
use std::collections::HashMap;

#[derive(Default)]
pub struct ActionKeeper {
    actions: HashMap<ActionName, Action>,
}

impl ActionKeeper {
    pub fn new(actions: HashMap<ActionName, Action>) -> Self {
        Self { actions }
    }
}

impl ActionKeeper {
    pub fn get(&self, name: &ActionName) -> RtResult<&Action> {
        self.actions.get(name).ok_or(RuntimeError::uex(format!(
            "the action {name} is not registered"
        )))
    }

    pub fn register(&mut self, name: ActionName, action: Action) -> RtResult<()> {
        &self.actions.insert(name, action);
        Ok(())
    }
}
