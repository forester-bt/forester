use crate::runtime::args::{RtArgs, RtArgument, RtValue};
use crate::runtime::context::{ChildIndex, RNodeState, TreeContext};
use crate::runtime::rtree::rnode::{FlowType, RNodeId};
use crate::runtime::{RtOk, RtResult, RuntimeError, TickResult};

pub const CURSOR: &str = "cursor";
pub const LEN: &str = "len";
pub const P_CURSOR: &str = "prev_cursor";

pub fn tick_run_with(tick_args: RtArgs, c: i64, l: i64) -> RtArgs {
    tick_args
        .with(CURSOR, RtValue::int(c))
        .with(LEN, RtValue::int(l))
}

pub(crate) fn read_len(args: RtArgs, def: i64) -> i64 {
    args.find(LEN.to_string())
        .and_then(|v| v.as_int())
        .unwrap_or(def)
}

pub(crate) fn read_cursor(tick_args: RtArgs) -> RtResult<i64> {
    Ok(tick_args
        .find(P_CURSOR.to_string())
        .or(tick_args.find(CURSOR.to_string()))
        .and_then(RtValue::as_int)
        .unwrap_or(0))
}

pub fn process(
    tpe: &FlowType,
    args: RtArgs,
    tick_args: RtArgs,
    res: TickResult,
    ctx: &mut TreeContext,
) -> RtResult<RNodeState> {
    match tpe {
        FlowType::Root => match res {
            r @ (TickResult::Success | TickResult::Failure(_)) => {
                Ok(RNodeState::fin(r, tick_run_with(tick_args, 0, 1)))
            }
            TickResult::Running => {
                ctx.next_tick()?;
                Ok(RNodeState::run(tick_run_with(tick_args, 0, 1)))
            }
        },
        FlowType::Sequence => {
            let cursor = read_cursor(tick_args.clone())?;
            let len = read_len(tick_args.clone(), 0);

            match res {
                TickResult::Failure(v) => Ok(RNodeState::fin(
                    TickResult::failure(v),
                    tick_run_with(tick_args.clone(), cursor, len),
                )),
                TickResult::Success => {
                    if cursor == len - 1 {
                        Ok(RNodeState::fin(
                            TickResult::success(),
                            tick_run_with(tick_args.clone(), cursor, len),
                        ))
                    } else {
                        Ok(RNodeState::run(tick_run_with(
                            tick_args.clone(),
                            cursor + 1,
                            len,
                        )))
                    }
                }
                TickResult::Running => Ok(RNodeState::run(tick_run_with(
                    tick_args.clone(),
                    cursor,
                    len,
                ))),
            }
        }
        FlowType::MSequence => {
            let cursor = read_cursor(tick_args.clone())?;
            let len = read_len(tick_args.clone(), 0);

            match res {
                TickResult::Failure(v) => Ok(RNodeState::fin(
                    TickResult::failure(v),
                    tick_run_with(
                        tick_args.clone().with(P_CURSOR, RtValue::int(cursor)),
                        cursor,
                        len,
                    ),
                )),
                TickResult::Success => {
                    if cursor == len - 1 {
                        Ok(RNodeState::fin(
                            TickResult::success(),
                            tick_run_with(tick_args.clone(), cursor, len),
                        ))
                    } else {
                        Ok(RNodeState::run(tick_run_with(
                            tick_args.clone(),
                            cursor + 1,
                            len,
                        )))
                    }
                }
                TickResult::Running => Ok(RNodeState::run(tick_run_with(
                    tick_args.clone(),
                    cursor,
                    len,
                ))),
            }
        }

        _ => Err(RuntimeError::UnImplementedAction("flow".to_string())), // FlowType::Parallel => {}
                                                                         // FlowType::MSequence => {}
                                                                         // FlowType::RSequence => {}
                                                                         // FlowType::Fallback => {}
                                                                         // FlowType::RFallback => {}
    }
}
