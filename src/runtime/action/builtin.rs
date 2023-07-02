pub mod data;

use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::blackboard::{BBKey, BlackBoard};
use crate::runtime::context::TreeContext;
use crate::runtime::{RuntimeError, TickResult};

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
    fn tick(&self, args: RtArgs, ctx: &mut TreeContext) -> Tick {
        Ok(match &self.res {
            TickResult::Failure(_) => {
                let mb_str = args.first();
                let c = match mb_str {
                    None => String::new(),
                    Some(v) => v.cast(ctx.bb()).string()?.unwrap_or_default(),
                };
                TickResult::failure(c)
            }
            r => r.clone(),
        })
    }
}
