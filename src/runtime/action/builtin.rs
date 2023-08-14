/// The actions that can help in implemeneting some logic.
/// Also, they are used in the std::actions
pub mod data;
pub mod http;
pub mod remote;

use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::RtArgs;
use crate::runtime::context::TreeContextRef;
use crate::runtime::TickResult;

/// Simple implementation to resturn result
pub struct ReturnResult {
    res: TickResult,
}

impl ReturnResult {
    pub fn success() -> ReturnResult {
        ReturnResult {
            res: TickResult::Success,
        }
    }
    pub fn fail(v: &str) -> ReturnResult {
        ReturnResult {
            res: TickResult::failure(v.to_string()),
        }
    }
    pub fn fail_empty() -> ReturnResult {
        ReturnResult {
            res: TickResult::failure_empty(),
        }
    }
    pub fn running() -> ReturnResult {
        ReturnResult {
            res: TickResult::Running,
        }
    }
}

impl Impl for ReturnResult {
    fn tick(&self, args: RtArgs, _ctx: TreeContextRef) -> Tick {
        Ok(match &self.res {
            TickResult::Failure(_) => {
                let mb_str = args.first();
                let c = match mb_str {
                    None => String::new(),
                    Some(v) => v.as_string().unwrap_or_default(),
                    // Some(v) => v.cast(ctx.bb()).string()?.unwrap_or_default(),
                };
                TickResult::failure(c)
            }
            r => r.clone(),
        })
    }
}
