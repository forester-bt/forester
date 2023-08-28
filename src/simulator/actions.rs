use crate::runtime::action::builtin::remote::RemoteHttpAction;
use crate::runtime::action::{Impl, ImplRemote, Tick};
use crate::runtime::args::RtArgs;
use crate::runtime::context::{TreeContextRef, TreeRemoteContextRef};
use crate::runtime::{RtResult, RuntimeError, TickResult};
use std::collections::HashMap;
use std::time::{Duration, SystemTime};

/// The action that represent the stubs fro the real actions.
pub enum SimAction {
    /// usize here is a millisecond for delay
    Success(usize),
    /// usize here is a millisecond for delay
    Random(usize),
    /// usize here is a millisecond for delay
    Failure(usize),
    /// The remote action wrapper
    Remote(RemoteHttpAction),
}

impl SimAction {
    pub fn is_remote(&self) -> bool {
        match self {
            SimAction::Remote(_) => true,
            _ => false,
        }
    }

    pub fn create(key: &str, params: HashMap<String, String>) -> RtResult<SimAction> {
        let delay = params
            .get("delay")
            .map(|s| s.parse::<usize>().unwrap_or_default())
            .unwrap_or_default();

        match key {
            "success" => Ok(SimAction::Success(delay)),
            "random" => Ok(SimAction::Random(delay)),
            "failure" => Ok(SimAction::Failure(delay)),
            "remote" => {
                let url = params.get("url").cloned().ok_or_else(|| {
                    RuntimeError::WrongArgument("the url is not specified".to_string())
                })?;

                let action = if let Some(serv) = params.get("server").cloned() {
                    RemoteHttpAction::new_with(url, serv)
                } else {
                    RemoteHttpAction::new(url)
                };

                Ok(SimAction::Remote(action))
            }
            e => Err(RuntimeError::WrongArgument(format!(
                "the {e} is not recognized as a simulation stub."
            ))),
        }
    }
}

impl ImplRemote for SimAction {
    fn tick(&self, args: RtArgs, ctx: TreeRemoteContextRef) -> Tick {
        match self {
            SimAction::Remote(delegate) => delegate.tick(args, ctx),
            _ => Err(RuntimeError::uex(
                "the remote action is expected here".to_string(),
            )),
        }
    }
}

impl Impl for SimAction {
    fn tick(&self, _args: RtArgs, _ctx: TreeContextRef) -> Tick {
        match self {
            SimAction::Success(d) => {
                std::thread::sleep(Duration::from_millis(*d as u64));
                Ok(TickResult::success())
            }
            SimAction::Failure(d) => {
                std::thread::sleep(Duration::from_millis(*d as u64));
                Ok(TickResult::failure_empty())
            }
            SimAction::Random(d) => {
                std::thread::sleep(Duration::from_millis(*d as u64));
                let num = SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap()
                    .as_millis();

                if num % 2 == 0 {
                    Ok(TickResult::success())
                } else {
                    Ok(TickResult::failure_empty())
                }
            }
            SimAction::Remote(_) => Ok(TickResult::Failure(
                "The remote action should execute another contract namely ImplRemote".to_string(),
            )),
        }
    }
}
