use std::future::Future;
use std::process::Output;
use std::sync::{Arc, Mutex};
use std::sync::atomic::AtomicBool;
use tokio::task::JoinHandle;
use tokio_util::sync::CancellationToken;
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
pub trait Daemon: Send + Sync {
    fn start(&mut self, ctx: DaemonContext);
    fn signal(&self) -> Arc<AtomicBool>;
}

pub enum DaemonTask {
    Unnamed(JoinHandle<()>, Arc<AtomicBool>),
    Named(DaemonName, JoinHandle<()>, Arc<AtomicBool>),
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
    pub fn cancel(&self) {
        match self {
            DaemonTask::Unnamed(_, t) |
            DaemonTask::Named(_, _, t) => t.store(true, std::sync::atomic::Ordering::Relaxed),
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

