use crate::runtime::action::Tick;
use crate::runtime::action::{Action, ActionName};
use crate::runtime::args::RtArgs;
use crate::runtime::context::{RNodeState, TreeContext};
use crate::runtime::env::RtEnv;
use crate::runtime::env::TaskState;
use crate::runtime::{RtResult, RuntimeError, TickResult};
use crate::tree::parser::ast::Tree;
use std::collections::HashMap;

/// Just a simple action map to register and execute the actions.
pub struct ActionKeeper {
    actions: HashMap<ActionName, Action>,
}

impl ActionKeeper {
    pub fn new(actions: HashMap<ActionName, Action>) -> RtResult<Self> {
        Ok(Self { actions })
    }
}

impl ActionKeeper {
    pub fn get_mut(&mut self, name: &ActionName) -> RtResult<&mut Action> {
        self.actions.get_mut(name).ok_or(RuntimeError::uex(format!(
            "the action {name} is not registered"
        )))
    }

    pub fn register(&mut self, name: ActionName, action: Action) -> RtResult<()> {
        &self.actions.insert(name, action);
        Ok(())
    }

    pub fn on_tick(
        &mut self,
        env: &mut RtEnv,
        name: &ActionName,
        args: RtArgs,
        ctx: &mut TreeContext,
    ) -> Tick {
        match self.get_mut(name)? {
            Action::Sync(action) => action.tick(args, ctx),
            Action::Async(ref mut action) => match env.task_state(name)? {
                TaskState::Absent => {
                    let action = action.clone();
                    let handle = env.runtime.spawn_blocking(move || action.tick(args));
                    env.tasks.insert(name.to_string(), handle);
                    Ok(TickResult::running())
                }
                TaskState::Started(handle) => {
                    env.tasks.insert(name.to_string(), handle);
                    Ok(TickResult::running())
                }
                TaskState::Finished(r) => r,
            },
        }
    }
}
