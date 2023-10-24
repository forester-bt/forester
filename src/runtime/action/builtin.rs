/// The actions that can help in implemeneting some logic.
/// Also, they are used in the std::actions
pub mod data;
pub mod http;
pub mod remote;
pub mod daemon;

use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::RtArgs;
use crate::runtime::context::TreeContextRef;
use crate::runtime::TickResult;

/// Simple implementation to return result
/// It is used in the std::actions also
pub struct ReturnResult {
    res: TickResult,
}

impl ReturnResult {
    /// Creates a new instance of the ReturnResult with the Success result
    pub fn success() -> ReturnResult {
        ReturnResult {
            res: TickResult::Success,
        }
    }

    /// Creates a new instance of the ReturnResult with the Failure result
    pub fn fail(v: &str) -> ReturnResult {
        ReturnResult {
            res: TickResult::failure(v.to_string()),
        }
    }
    /// Creates a new instance of the ReturnResult with the Failure result
    pub fn fail_empty() -> ReturnResult {
        ReturnResult {
            res: TickResult::failure_empty(),
        }
    }
    /// Creates a new instance of the ReturnResult with the Running result
    pub fn running() -> ReturnResult {
        ReturnResult {
            res: TickResult::Running,
        }
    }
}

impl Impl for ReturnResult {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        Ok(match &self.res {
            TickResult::Failure(_) => {
                let mb_str = args.first();
                let c = match mb_str {
                    None => String::new(),
                    Some(v) => v.cast(ctx).str()?.unwrap_or_default(),
                };
                TickResult::failure(c)
            }
            r => r.clone(),
        })
    }
}
