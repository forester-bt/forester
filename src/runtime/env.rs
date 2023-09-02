use crate::runtime::action::ActionName;
use crate::runtime::action::Tick;
use crate::runtime::{RtResult, RuntimeError};
use std::collections::HashMap;
use std::future::IntoFuture;
use tokio::runtime::{Builder, Runtime};
use tokio::task::JoinError;
use tokio::task::JoinHandle;

/// Runtime to execute async tasks.
/// It is not supposed to be used directly (except maybe for remote action execution)
/// Based on the tokio Runtime
pub struct RtEnv {
    pub runtime: Runtime,
    pub tasks: HashMap<ActionName, JoinHandle<Tick>>,
}

impl From<JoinError> for RuntimeError {
    fn from(value: JoinError) -> Self {
        RuntimeError::fail(value.to_string())
    }
}

pub enum TaskState {
    Absent,
    Started(JoinHandle<Tick>),
    Finished(Tick),
}

impl RtEnv {
    pub fn new(runtime: Runtime) -> Self {
        Self {
            runtime,
            tasks: HashMap::default(),
        }
    }
    pub fn try_new() -> RtResult<Self> {
        let runtime = Builder::new_multi_thread().enable_all().build()?;
        Ok(Self {
            runtime,
            tasks: HashMap::default(),
        })
    }
}

impl RtEnv {
    pub fn task_state(&mut self, name: &ActionName) -> RtResult<TaskState> {
        match self.tasks.remove(name) {
            None => Ok(TaskState::Absent),
            Some(jh) if !jh.is_finished() => Ok(TaskState::Started(jh)),
            Some(jh) => Ok(TaskState::Finished(
                self.runtime.block_on(jh.into_future())?,
            )),
        }
    }
}
