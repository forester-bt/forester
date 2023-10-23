pub mod daemon;

use crate::runtime::action::ActionName;
use crate::runtime::action::Tick;
use crate::runtime::{RtOk, RtResult, RuntimeError};
use std::collections::HashMap;
use std::future::IntoFuture;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use itertools::Itertools;
use tokio::runtime::{Builder, Runtime};
use tokio::select;
use tokio::task::JoinError;
use tokio::task::JoinHandle;
use tokio_util::sync::CancellationToken;
use crate::runtime::env::daemon::{Daemon, DaemonContext, DaemonName, DaemonTask};

/// Runtime to execute async tasks.
/// It is not supposed to be used directly (except maybe for remote action execution)
/// Based on the tokio Runtime
pub struct RtEnv {
    pub runtime: Runtime,
    pub tasks: HashMap<ActionName, JoinHandle<Tick>>,
    pub daemons: Vec<DaemonTask>,
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
            daemons: Vec::default(),
        }
    }
    pub fn try_new() -> RtResult<Self> {
        let runtime = Builder::new_multi_thread().enable_all().build()?;
        Ok(Self {
            runtime,
            tasks: HashMap::default(),
            daemons: Vec::default(),
        })
    }
    fn start_daemon_impl(&mut self, mut daemon: Box<dyn Daemon>, ctx: DaemonContext) -> RtResult<(JoinHandle<()>, Arc<AtomicBool>)> {
        let flag = daemon.signal();
        Ok(
            (self.runtime.spawn(async move {
                daemon.start(ctx)
            }), flag)
        )
    }
    pub fn start_daemon(&mut self, daemon: Box<dyn Daemon>, ctx: DaemonContext) -> RtOk {
        debug!(target:"daemon","start an unnamed daemon");
        let (jh, t) = self.start_daemon_impl(daemon, ctx)?;
        let task = DaemonTask::Unnamed(jh, t);
        self.daemons.push(task);
        Ok(())
    }
    pub fn start_named_daemon(&mut self, name: DaemonName, daemon: Box<dyn Daemon>, ctx: DaemonContext) -> RtOk {
        let (jh, t) = self.start_daemon_impl(daemon, ctx)?;
        debug!(target:"daemon","start a daemon {}",name);
        let task = DaemonTask::Named(name, jh, t);
        self.daemons.push(task);
        Ok(())
    }

    /// tries to stop the demon by name.
    pub fn stop_daemon(&mut self, name: &DaemonName) {
        let mb_idx =
            self.daemons.iter()
                .find_position(|n| n.name().filter(|n| *n == name).is_some())
                .map(|(i, _)| i);
        if let Some(idx) = mb_idx {
            debug!(target:"daemon","stop a daemon {}, found at the {}",name,idx);
            let task = self.daemons.remove(idx);
            task.cancel();
        } else {
            debug!(target:"daemon","stop a daemon {}, but the daemon not found",name);
        }
    }

    pub fn stop_all_daemons(&mut self) {
        self.daemons.iter().for_each(|task| {
            debug!(target:"daemon","stop a daemon {}",task.name().unwrap_or(&"unnamed".to_string()));
            task.cancel();
        });
        self.daemons.clear();
    }

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

