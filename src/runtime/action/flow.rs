use crate::runtime::args::RtArgs;
use crate::runtime::context::{ChildIndex, RNodeState, TreeContext};
use crate::runtime::rtree::rnode::FlowType;
use crate::runtime::{RtOk, RtResult, RuntimeError, TickResult};

pub fn handle_flow(
    tpe: &FlowType,
    args: RtArgs,
    child_res: TickResult,
    cursor: ChildIndex,
    child_len: usize,
    ctx: &mut TreeContext,
) -> RtResult<RNodeState> {
    match tpe {
        FlowType::Root => match child_res {
            r @ (TickResult::Success | TickResult::Failure(_)) => {
                Ok(RNodeState::fin(r, cursor, child_len))
            }
            TickResult::Running => {
                ctx.shift()?;
                Ok(RNodeState::running(cursor, child_len))
            }
        },
        _ => Err(RuntimeError::UnImplementedAction("flow".to_string())), // FlowType::Parallel => {}
                                                                         // FlowType::Sequence => {}
                                                                         // FlowType::MSequence => {}
                                                                         // FlowType::RSequence => {}
                                                                         // FlowType::Fallback => {}
                                                                         // FlowType::RFallback => {}
    }
}
