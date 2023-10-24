use Ordering::Relaxed;
use std::future::Future;
use std::process::Output;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use tokio::task::JoinHandle;
use crate::runtime::blackboard::{BBRef, BlackBoard};
use crate::runtime::context::{TracerRef, TreeContext};

/// the name of the daemon
pub type DaemonName = String;

/// the signal to stop the daemon. Just a boolean flag
/// that is initially false and can be set to true to stop the daemon
pub type StopSignal = Arc<AtomicBool>;


/// A trait for a daemon
/// A daemon is a process that runs in the background
/// It is not supposed to be used directly
/// It can solve the problems like:
/// - sync a blackboard with the external sources
/// - perform some actions periodically
///
/// The trait provides 2 methods:
/// - perform - the method that is called when the daemon is started
/// - signal - the method that returns the signal to stop the daemon
///
/// # Notes
/// The trait provides sync methods but they will likely run in the async environment.
/// Most likely the daemon will implement infinite loop inside the start method
/// that is very important to handle the signal to stop the daemon correctly.
///
/// # Example
/// ```
/// use std::sync::atomic::Ordering::Relaxed;
/// use forester_rs::runtime::args::RtValue;
/// use forester_rs::runtime::blackboard;
/// use forester_rs::runtime::env::daemon::{Daemon, DaemonContext, StopSignal};
/// struct Test(StopSignal);
///
/// impl Daemon for Test {
///     fn perform(&mut self, ctx: DaemonContext) {
///        while !self.signal().load(Relaxed) {
///             std::thread::sleep(std::time::Duration::from_millis(500));
///             blackboard::utils::push_to_arr(ctx.bb.clone(), "test".into(), RtValue::int(1)).unwrap();
///         }
///     }
///
///     fn signal(&self) -> StopSignal {
///         self.0.clone()
///     }
/// }
/// ```
pub trait Daemon: Send + Sync {
    /// The method that is called when the daemon is started.
    /// The method will be executed in the async environment.
    ///
    /// # Note
    /// The method is supposed to be an infinite loop that will be stopped
    /// when the signal is set to true.
    /// **So it is important to check the signal in the loop**
    fn perform(&mut self, ctx: DaemonContext);

    /// The method that returns the signal to stop the daemon.
    /// The signal is just a boolean flag that is initially false
    fn signal(&self) -> StopSignal;
}

/// The task that is executed in the async environment
pub enum DaemonTask {
    Unnamed(JoinHandle<()>, StopSignal),
    Named(DaemonName, JoinHandle<()>, StopSignal),
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
            DaemonTask::Named(_, _, t) => t.store(true, Relaxed),
        }
    }
}

/// The context for the daemon.
#[derive(Clone, Default, Debug)]
pub struct DaemonContext {
    pub bb: BBRef,
    pub tracer: TracerRef,
}

impl DaemonContext {
    pub fn new(bb: BBRef, tracer: TracerRef) -> Self {
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

