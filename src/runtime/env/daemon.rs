pub mod context;
pub mod task;

use std::future::Future;
use std::pin::Pin;
use std::sync::{Arc};
use std::sync::atomic::{AtomicBool};

use tokio_util::sync::CancellationToken;
use crate::runtime::env::daemon::context::DaemonContext;

/// the name of the daemon
pub type DaemonName = String;

/// the signal to stop the daemon. Just a boolean flag
/// that is initially false and can be set to true to stop the daemon
pub type StopFlag = Arc<AtomicBool>;


/// A trait to implement a daemon function
/// A daemon is a process that runs in the background
/// It is not supposed to be used directly
/// It can solve the problems like:
/// - sync a blackboard with the external sources
/// - perform some actions periodically
///
/// The trait provides 2 methods:
/// - perform - the method that is called when the daemon is started
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
/// use forester_rs::runtime::env::daemon::{DaemonFn, StopFlag};
/// use forester_rs::runtime::env::daemon::context::DaemonContext;
/// struct Test;
///
/// impl DaemonFn for Test {
///     fn perform(&mut self, ctx: DaemonContext,signal:StopFlag) {
///        while !signal.load(Relaxed) {
///             std::thread::sleep(std::time::Duration::from_millis(500));
///             blackboard::utils::push_to_arr(ctx.bb.clone(), "test".into(), RtValue::int(1)).unwrap();
///         }
///     }
/// }
/// ```
pub trait DaemonFn: Send + Sync {
    /// The method that is called when the daemon is started.
    /// The method will be executed in the async environment.
    ///
    /// # Note
    /// The method is supposed to be an infinite loop that will be stopped
    /// when the signal is set to true.
    /// **So it is important to check the signal in the loop**
    fn perform(&mut self, ctx: DaemonContext, signal: StopFlag);
}

/// A trait to implement an async daemon function
/// A daemon is a process that runs in the background
/// It is not supposed to be used directly
/// It can solve the problems like:
/// - sync a blackboard with the external sources
/// - perform some actions periodically
/// - perform some actions asynchronously
/// - perform some actions in the separate thread
///
/// The trait provides a method to prepare a returning async function
///
/// # Example
/// ```
/// use std::future::Future;
/// use std::pin::Pin;
/// use tokio_util::sync::CancellationToken;
/// use forester_rs::runtime::env::daemon::AsyncDaemonFn;
/// use forester_rs::runtime::env::daemon::context::DaemonContext;
/// use forester_rs::runtime::args::RtValue;
/// struct BBSync;
/// impl AsyncDaemonFn for BBSync {
///     fn prepare(&mut self, ctx: DaemonContext, signal: CancellationToken) -> Pin<Box<dyn Future<Output=()> + Send>> {
///         Box::pin(async move {
///             loop {
///                 tokio::select! {
///                 _ = signal.cancelled() => {
///                     return;
///                 }
///                 _ = tokio::time::sleep(std::time::Duration::from_millis(10)) => {
///                     let mut bb = ctx.bb.lock().unwrap();
///                     let v = bb.get("test".to_string()).expect("no errors")
///                         .cloned().unwrap_or(RtValue::int(0));
///
///                     bb.put("test_daemon".to_string(), v).unwrap();
///                 }
///               }
///             }
///         })
///     }
/// }
/// ```
pub trait AsyncDaemonFn: Send + Sync {
    /// Prepare the async function that will be executed in the async environment
    /// it receives the context and the signal to stop the daemon
    fn prepare(&mut self, ctx: DaemonContext, signal: CancellationToken) -> Pin<Box<dyn Future<Output=()> + Send>>;
}

/// The daemon that can be executed in the async environment in the background
pub enum Daemon {
    Sync(Box<dyn DaemonFn>),
    Async(Box<dyn AsyncDaemonFn>),
}

impl Daemon {
    pub fn sync<T>(daemon: T) -> Self where T: DaemonFn + 'static {
        Self::Sync(Box::new(daemon))
    }
    pub fn a_sync<T>(daemon: T) -> Self where T: AsyncDaemonFn + 'static {
        Self::Async(Box::new(daemon))
    }
}




