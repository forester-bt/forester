use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::RtArgs;
use crate::runtime::context::TreeContext;
use crate::runtime::{RuntimeError, TickResult};

struct Empty(String);

impl Impl for Empty {
    fn tick(&self, _args: RtArgs, _ctx: &mut TreeContext) -> Tick {
        Err(RuntimeError::UnImplementedAction(self.0.to_string()))
    }
}

pub struct Fail;

impl Impl for Fail {
    fn tick(&self, args: RtArgs, _ctx: &mut TreeContext) -> Tick {
        let c = {
            if args.0.len() == 1 {
                args.0
                    .first()
                    .and_then(|v| v.clone().val().as_string())
                    .unwrap_or("fail".to_string())
            } else {
                args.find("reason".to_string())
                    .and_then(|v| v.as_string())
                    .unwrap_or("fail".to_string())
            }
        };
        println!("fail -> {}", c);
        Ok(TickResult::failure(c))
    }
}

struct Print;

impl Impl for Print {
    fn tick(&self, args: RtArgs, ctx: &mut TreeContext) -> Tick {
        println!("{:?}", args);
        Ok(TickResult::success())
    }
}
