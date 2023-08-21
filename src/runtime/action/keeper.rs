use crate::runtime::action::Tick;
use crate::runtime::action::{Action, ActionName};
use crate::runtime::args::RtArgs;
use crate::runtime::context::{TreeContextRef, TreeRemoteContextRef};
use crate::runtime::env::RtEnv;
use crate::runtime::env::TaskState;
use crate::runtime::forester::serv::ServInfo;
use crate::runtime::{RtResult, RuntimeError, TickResult};
use std::collections::{HashMap, HashSet};

/// Just a simple action map to register and execute the actions.
pub struct ActionKeeper {
    actions: HashMap<ActionName, ActionImpl>,
}

pub enum ActionImpl {
    Present(Action),
    Absent,
}

impl ActionImpl {
    pub fn action(&mut self) -> Option<&mut Action> {
        match self {
            ActionImpl::Present(a) => Some(a),
            ActionImpl::Absent => None,
        }
    }
    pub fn is_absent(&self) -> bool {
        matches!(self, ActionImpl::Absent)
    }
}

impl ActionKeeper {
    pub fn actions(&self) -> HashSet<&ActionName> {
        self.actions.keys().collect()
    }

    pub fn new_with<T>(
        impl_actions: HashMap<ActionName, Action>,
        all_actions: HashSet<ActionName>,
        default: T,
    ) -> RtResult<Self>
    where
        T: Fn() -> ActionImpl,
    {
        let mut impl_actions = impl_actions;
        let mut actions = HashMap::new();
        for action_name in all_actions {
            if let Some(a) = impl_actions.remove(&action_name) {
                debug!(target:"action","register action {action_name} with the given impl");
                actions.insert(action_name, ActionImpl::Present(a));
            } else {
                debug!(target:"action","register action {action_name} with the default impl");
                let action_impl = default();
                if action_impl.is_absent() {
                    debug!(target:"action","The action {action_name} is absent and the execution will be failed on calling this action.");
                }
                actions.insert(action_name, action_impl);
            }
        }

        Ok(Self { actions })
    }
    fn get_mut(&mut self, name: &ActionName) -> RtResult<&mut Action> {
        self.actions
            .get_mut(name)
            .and_then(|t| t.action())
            .ok_or(RuntimeError::uex(format!(
                "the action {name} is not registered"
            )))
    }

    pub fn register(&mut self, name: ActionName, action: Action) -> RtResult<()> {
        debug!(target:"action","A new action {name} is registered");
        let _ = self.actions.insert(name, ActionImpl::Present(action));
        Ok(())
    }

    /// Execute an action, previously find it by name.
    /// If the action is async and running, check the process instead.
    pub fn on_tick(
        &mut self,
        env: &mut RtEnv,
        name: &ActionName,
        args: RtArgs,
        ctx: TreeContextRef,
        http_serv: &Option<ServInfo>,
    ) -> Tick {
        match self.get_mut(name)? {
            Action::Sync(action) => action.tick(args, ctx),
            Action::Remote(action) => action.tick(
                args,
                TreeRemoteContextRef::new(ctx.current_tick(), get_port(http_serv)?, env),
            ),
            Action::Async(ref mut action) => match env.task_state(name)? {
                TaskState::Absent => {
                    let action = action.clone();
                    env.tasks.insert(
                        name.to_string(),
                        env.runtime.spawn_blocking(move || action.tick(args, ctx)),
                    );
                    Ok(TickResult::running())
                }
                TaskState::Started(handle) => {
                    // return it to the running tasks
                    env.tasks.insert(name.to_string(), handle);
                    Ok(TickResult::running())
                }
                TaskState::Finished(r) => r,
            },
        }
    }
}
fn get_port(http_serv: &Option<ServInfo>) -> Result<u16, RuntimeError> {
    http_serv
        .as_ref()
        .map(|v| v.serv_port)
        .filter(|p| *p > 0)
        .ok_or(RuntimeError::RecoveryToFailure(
            format!("the http server port is not found or incorrect").to_string(),
        ))
}
