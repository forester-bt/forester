use crate::runtime::action::builtin::remote::RemoteHttpAction;
use crate::runtime::action::{Impl, ImplRemote, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::{TreeContextRef, TreeRemoteContextRef};
use crate::runtime::{RtOk, RtResult, RuntimeError, TickResult};
use std::collections::HashMap;
use std::time::{Duration, SystemTime};

/// The action that represent the stubs fro the real actions.
pub enum SimAction {
    /// usize here is a millisecond for delay, the map is the key-value pairs
    /// written to the blackboard on every tick.
    Success(usize, HashMap<String, RtValue>),
    /// usize here is a millisecond for delay, the map is the key-value pairs
    /// written to the blackboard on every tick.
    Random(usize, HashMap<String, RtValue>),
    /// usize here is a millisecond for delay, the map is the key-value pairs
    /// written to the blackboard on every tick.
    Failure(usize, HashMap<String, RtValue>),
    /// The remote action wrapper
    Remote(RemoteHttpAction),
}

impl SimAction {
    pub fn is_remote(&self) -> bool {
        matches!(self, SimAction::Remote(_))
    }

    pub fn create(
        key: &str,
        params: HashMap<String, String>,
        bb: HashMap<String, RtValue>,
    ) -> RtResult<SimAction> {
        let delay = params
            .get("delay")
            .map(|s| s.parse::<usize>().unwrap_or_default())
            .unwrap_or_default();

        match key {
            "success" => Ok(SimAction::Success(delay, bb)),
            "random" => Ok(SimAction::Random(delay, bb)),
            "failure" => Ok(SimAction::Failure(delay, bb)),
            "remote" => {
                let url = params.get("url").cloned().ok_or_else(|| {
                    RuntimeError::WrongArgument("the url is not specified".to_string())
                })?;

                Ok(SimAction::Remote(RemoteHttpAction::new(url)))
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

fn write_bb(bb: &HashMap<String, RtValue>, ctx: TreeContextRef) -> RtOk {
    let bb_ref = ctx.bb();
    let mut store = bb_ref.lock()?;
    for (key, value) in bb {
        store.put(key.clone(), value.clone())?;
    }
    Ok(())
}

impl Impl for SimAction {
    fn tick(&self, _args: RtArgs, ctx: TreeContextRef) -> Tick {
        match self {
            SimAction::Success(d, bb) => {
                write_bb(bb, ctx)?;
                std::thread::sleep(Duration::from_millis(*d as u64));
                Ok(TickResult::success())
            }
            SimAction::Failure(d, bb) => {
                write_bb(bb, ctx)?;
                std::thread::sleep(Duration::from_millis(*d as u64));
                Ok(TickResult::failure_empty())
            }
            SimAction::Random(d, bb) => {
                write_bb(bb, ctx)?;
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

#[cfg(test)]
mod tests {
    use crate::runtime::action::Impl;
    use crate::runtime::args::{RtArgs, RtValue};
    use crate::runtime::blackboard::BlackBoard;
    use crate::runtime::context::TreeContextRef;
    use crate::runtime::env::RtEnv;
    use crate::runtime::trimmer::TrimmingQueue;
    use crate::runtime::TickResult;
    use crate::simulator::actions::SimAction;
    use crate::tracer::Tracer;
    use std::collections::HashMap;
    use std::sync::{Arc, Mutex};

    #[test]
    fn smoke() {
        let action = SimAction::Success(0, HashMap::default());
        let result = action.tick(
            RtArgs(vec![]),
            TreeContextRef::new(
                Arc::new(Mutex::new(BlackBoard::default())),
                Arc::new(Mutex::new(Tracer::default())),
                0,
                Arc::new(Mutex::new(TrimmingQueue::default())),
                Arc::new(Mutex::new(RtEnv::try_new().unwrap())),
            ),
        );

        assert!(result.is_ok());
        assert_eq!(result.unwrap(), TickResult::Success);
    }

    #[test]
    fn writes_bb() {
        let bb = Arc::new(Mutex::new(BlackBoard::default()));
        let action = SimAction::Success(
            0,
            HashMap::from_iter(vec![
                ("a".to_string(), RtValue::int(10)),
                ("b".to_string(), RtValue::str("a".to_string())),
            ]),
        );

        let result = action.tick(
            RtArgs(vec![]),
            TreeContextRef::new(
                bb.clone(),
                Arc::new(Mutex::new(Tracer::default())),
                0,
                Arc::new(Mutex::new(TrimmingQueue::default())),
                Arc::new(Mutex::new(RtEnv::try_new().unwrap())),
            ),
        );

        assert_eq!(result.unwrap(), TickResult::Success);

        let store = bb.lock().unwrap();
        assert_eq!(store.get("a".to_string()).unwrap(), Some(&RtValue::int(10)));
        assert_eq!(
            store.get("b".to_string()).unwrap(),
            Some(&RtValue::str("a".to_string()))
        );
    }
}
