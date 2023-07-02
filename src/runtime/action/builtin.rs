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
    fn tick(&self, args: RtArgs, _ctx: &mut TreeContext) -> Tick {
        Ok(match &self.res {
            TickResult::Failure(_) => {
                TickResult::failure(args.first_as(RtValue::as_string).unwrap_or("".to_string()))
            }
            r => r.clone(),
        })
    }
}
