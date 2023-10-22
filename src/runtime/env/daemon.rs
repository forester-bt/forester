use std::sync::{Arc, Mutex};
use tokio::task::JoinHandle;
use crate::runtime::{RtOk};
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::context::TreeContext;
use crate::tracer::Tracer;


pub type DaemonName = String;

/// A trait for a daemon
/// A daemon is a process that runs in the background
/// It is not supposed to be used directly
/// It can solve the problems like:
/// - sync a blackboard with the external sources
/// - perform some actions periodically
pub trait Daemon:Sync + Send {
    fn start(&mut self, ctx: DaemonContext) -> RtOk;
}
pub enum DaemonTask {
    Unnamed(JoinHandle<RtOk>),
    Named(DaemonName, JoinHandle<RtOk>),
}

impl DaemonTask {
    pub fn name(&self) -> Option<&DaemonName> {
        match self {
            DaemonTask::Unnamed(_) => None,
            DaemonTask::Named(name, _) => Some(name),
        }
    }
    pub fn jh(&self) -> &JoinHandle<RtOk> {
        match self {
            DaemonTask::Unnamed(jh) => jh,
            DaemonTask::Named(_, jh) => jh,
        }
    }
}

#[derive(Clone)]
pub struct DaemonContext {
    pub bb: Arc<Mutex<BlackBoard>>,
    pub tracer: Arc<Mutex<Tracer>>,
}

impl DaemonContext {
    pub fn new(bb: Arc<Mutex<BlackBoard>>, tracer: Arc<Mutex<Tracer>>) -> Self {
        Self { bb, tracer }
    }
}

impl From<TreeContext> for DaemonContext {
    fn from(mut value: TreeContext) -> Self {
        DaemonContext {
            bb: value.bb(),
            tracer: value.tracer(),
        }
    }
}

#[cfg(test)]
mod tests {

    #[test]
    fn smoke(){




    }

}