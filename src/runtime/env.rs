pub mod daemon;

use crate::runtime::action::ActionName;
use crate::runtime::action::Tick;
use crate::runtime::{RtOk, RtResult, RuntimeError};
use std::collections::HashMap;
use std::future::IntoFuture;
use std::sync::{Arc, Mutex};
use std::sync::atomic::AtomicBool;
use itertools::Itertools;
use tokio::runtime::{Builder, Runtime};

use tokio::task::JoinError;
use tokio::task::JoinHandle;
use tokio_util::sync::CancellationToken;
use crate::runtime::env::daemon::{DaemonFn, DaemonName, Daemon};
use crate::runtime::env::daemon::context::DaemonContext;
use crate::runtime::env::daemon::task::{DaemonStopSignal, DaemonTask};


pub type RtEnvRef = Arc<Mutex<RtEnv>>;

/// Runtime to execute async tasks.
/// It is not supposed to be used directly (except maybe for remote action execution)
/// or starting daemons.
/// Based on the tokio Runtime
pub struct RtEnv {
    /// The tokio runtime
    pub runtime: Runtime,
    /// The async tasks
    pub tasks: HashMap<ActionName, JoinHandle<Tick>>,
    /// The daemons
    pub daemons: Vec<DaemonTask>,
}

impl Default for RtEnv {
    fn default() -> Self {
        RtEnv::try_new().unwrap()
    }
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
    fn start_daemon_impl(&mut self, daemon: Daemon, ctx: DaemonContext) -> RtResult<(JoinHandle<()>, DaemonStopSignal)> {
        Ok(
            match daemon {
                Daemon::Sync(mut f) => {
                    let signal = Arc::new(AtomicBool::new(false));
                    let ret_signal = DaemonStopSignal::Sync(signal.clone());
                    let jh = self.runtime.spawn(async move {
                        f.perform(ctx.into(), signal);
                    });

                    (jh, ret_signal)
                }
                Daemon::Async(mut f) => {
                    let token = CancellationToken::new();
                    let token_rv = token.clone();
                    let handle = self.runtime.spawn(
                        f.prepare(ctx.into(), token_rv)
                    );
                    (handle, DaemonStopSignal::Async(token))
                }
            }
        )
    }
    /// start a daemon
    /// Params:
    /// - daemon: the daemon to start
    /// - ctx: the daemon context. Can be obtained from the tree context
    pub fn start_daemon(&mut self, daemon: Daemon, ctx: DaemonContext) -> RtOk {
        debug!(target:"daemon","start an unnamed daemon");
        let (jh, t) = self.start_daemon_impl(daemon, ctx)?;
        let task = DaemonTask::Unnamed(jh, t);
        self.daemons.push(task);
        Ok(())
    }

    /// start a daemon with the given name
    /// Params:
    /// - name: the name of the daemon
    /// - daemon: the daemon to start
    /// - ctx: the daemon context. Can be obtained from the tree context
    ///
    /// The name is used to stop the daemon
    pub fn start_named_daemon(&mut self, name: DaemonName, daemon: Daemon, ctx: DaemonContext) -> RtOk {
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
            let mut task = self.daemons.remove(idx);
            if let Some(e) = task.stop().err() {
                debug!(target:"daemon","the daemon can not be stopped due to {:?}",e);
            }
        } else {
            debug!(target:"daemon","stop a daemon {}, but the daemon not found",name);
        }
    }

    /// check if the daemon is running
    pub fn daemon_is_running(&self, name: &DaemonName) -> RtResult<bool> {
        self.daemons.iter()
            .find(|d| d.name().filter(|n| *n == name).is_some())
            .map(|t| !t.jh().is_finished())
            .ok_or(RuntimeError::fail(format!("the daemon {} is not found", name)))
    }

    /// stop all daemons
    pub fn stop_all_daemons(&mut self) {
        for mut task in self.daemons.drain(..) {
            debug!(target:"daemon","stop a daemon {}",task.name().unwrap_or(&"unnamed".to_string()));
            if let Some(e) = task.stop().err() {
                debug!(target:"daemon","the daemon can not be stopped due to {:?}",e);
            }
        }
    }

    /// the state of the async task
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

