use std::sync::atomic::Ordering::Relaxed;

use tokio::task::JoinHandle;
use tokio_util::sync::CancellationToken;
use crate::runtime::env::daemon::{DaemonName, StopFlag};
use crate::runtime::{RtOk};

pub enum DaemonTask {
    Unnamed(JoinHandle<()>, DaemonStopSignal),
    Named(DaemonName, JoinHandle<()>, DaemonStopSignal),
}

pub enum DaemonStopSignal {
    Sync(StopFlag),
    Async(CancellationToken),
}

impl DaemonStopSignal {
    pub fn stop(&self) -> RtOk {
        match self {
            DaemonStopSignal::Sync(s) => {
                s.store(true, Relaxed);
            }
            DaemonStopSignal::Async(s) => {
                s.cancel();
            }
        }
        Ok(())
    }
}

impl DaemonTask {
    pub fn name(&self) -> Option<&DaemonName> {
        match self {
            DaemonTask::Unnamed(_, _) => None,
            DaemonTask::Named(name, _, _) => Some(name),
        }
    }
    pub fn jh(&self) -> &JoinHandle<()> {
        match self {
            DaemonTask::Unnamed(jh, _) => jh,
            DaemonTask::Named(_, jh, _) => jh,
        }
    }
    pub fn stop(&mut self) -> RtOk {
        match self {
            DaemonTask::Unnamed(_, t) |
            DaemonTask::Named(_, _, t) => t.stop(),
        }
    }
}