use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::RtArgs;
use crate::runtime::context::TreeContextRef;
use crate::runtime::{RtResult, RuntimeError, TickResult};
use std::time::{Duration, SystemTime};

/// The action that represent the stubs fro the real actions.
pub enum SimAction {
    /// usize here is a miilisecond for delay
    Success(usize),
    /// usize here is a miilisecond for delay
    Random(usize),
    /// usize here is a miilisecond for delay
    Failure(usize),
}

impl SimAction {
    pub fn create(key: &str, delay: usize) -> RtResult<SimAction> {
        match key {
            "success" => Ok(SimAction::Success(delay)),
            "random" => Ok(SimAction::Random(delay)),
            "failure" => Ok(SimAction::Failure(delay)),
            e => Err(RuntimeError::WrongArgument(format!(
                "the {e} is not recognized as simulation stub."
            ))),
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
        }
    }
}
