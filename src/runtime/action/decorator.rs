use crate::runtime::action::Tick;
use crate::runtime::args::RtArgs;
use crate::runtime::context::{RNodeState, TreeContext};
use crate::runtime::rtree::rnode::DecoratorType;
use crate::runtime::{RtOk, RtResult, RuntimeError, TickResult};

pub fn handle_decorator(
    tpe: &DecoratorType,
    run_args: RtArgs,
    init_args: RtArgs,
    child_res: TickResult,
    ctx: &mut TreeContext,
) -> RtResult<RNodeState> {
    match tpe {
        DecoratorType::Inverter => match child_res {
            TickResult::Success => Ok(RNodeState::fin(
                TickResult::failure("decorator inverts the result.".to_string()),
                1,
                1,
            )),
            TickResult::Failure(_) => Ok(RNodeState::fin(TickResult::success(), 0, 1)),
            TickResult::Running => Ok(RNodeState::running(0, 1)),
        },
        _ => Err(RuntimeError::UnImplementedAction("decorator".to_string())), // DecoratorType::ForceSuccess => {}
                                                                              // DecoratorType::ForceFail => {}
                                                                              // DecoratorType::Repeat => {}
                                                                              // DecoratorType::Retry => {}
                                                                              // DecoratorType::Timeout => {}
                                                                              // DecoratorType::Delay => {}
    }
}
