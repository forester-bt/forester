use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::RtArgs;
use crate::runtime::context::TreeContextRef;
use crate::runtime::{RuntimeError, TickResult};

use crate::tracer::Event;


fn daemon_name(args: RtArgs, ctx: &TreeContextRef) -> Result<String, RuntimeError> {
    args.first()
        .ok_or(RuntimeError::fail(
            "the name is expected and should be a string".to_string(),
        ))
        .and_then(|v| v.cast(ctx.clone()).str())
        .and_then(|v| {
            v.ok_or(RuntimeError::fail(
                "the name is expected and should be a string".to_string(),
            ))
        })
}

/// Stops the daemon by name
pub struct StopDaemonAction;

impl Impl for StopDaemonAction {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let daemon_name = daemon_name(args, &ctx)?;
        ctx.env().lock()?.stop_daemon(&daemon_name);
        ctx.trace_ev(Event::Daemon(format!("stop {}", daemon_name)))?;
        Ok(TickResult::Success)
    }
}

/// Checks if the daemon is running
pub struct CheckDaemonAction;

impl Impl for CheckDaemonAction {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let daemon_name = daemon_name(args, &ctx)?;
        ctx
            .env()
            .lock()?
            .daemon_is_running(&daemon_name)
            .map(|r| {
                if r { TickResult::success() } else { TickResult::failure_empty() }
            })
    }
}
