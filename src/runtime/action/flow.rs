use crate::runtime::args::{RtArgs, RtArgument, RtValue};
use crate::runtime::context::{ChildIndex, RNodeState, TreeContext};
use crate::runtime::rtree::rnode::{FlowType, RNodeId};
use crate::runtime::{RtOk, RtResult, RuntimeError, TickResult};

// current child
pub const CURSOR: &str = "cursor";
// the child len
pub const LEN: &str = "len";

// the current cursor
// when the process is torn up(the child returns running or in seq is failure etc)
pub const P_CURSOR: &str = "prev_cursor";
// reason for the failure
pub const REASON: &str = "reason";

pub fn run_with(tick_args: RtArgs, c: i64, l: i64) -> RtArgs {
    tick_args
        .with(CURSOR, RtValue::int(c))
        .with(LEN, RtValue::int(l))
}

pub(crate) fn read_len_or_zero(args: RtArgs) -> i64 {
    args.find(LEN.to_string())
        .and_then(|v| v.as_int())
        .unwrap_or(0)
}

pub(crate) fn read_cursor(tick_args: RtArgs) -> RtResult<i64> {
    Ok(tick_args
        .find(P_CURSOR.to_string())
        .or(tick_args.find(CURSOR.to_string()))
        .and_then(RtValue::as_int)
        .unwrap_or(0))
}
/// Shortest version of TickResult, containing only finished statuses.
pub enum TickResultFin {
    Failure(String),
    Success,
}

impl TryFrom<RNodeState> for TickResultFin {
    type Error = RuntimeError;

    fn try_from(value: RNodeState) -> Result<Self, Self::Error> {
        match value {
            RNodeState::Success(_) => Ok(TickResultFin::Success),
            RNodeState::Failure(v) => {
                let r = v
                    .find(REASON.to_string())
                    .and_then(RtValue::as_string)
                    .unwrap_or_default();
                Ok(TickResultFin::Failure(r))
            }
            _ => Err(RuntimeError::uex("running is unexpected".to_string())),
        }
    }
}
impl Into<TickResult> for TickResultFin {
    fn into(self) -> TickResult {
        match self {
            TickResultFin::Failure(v) => TickResult::Failure(v),
            TickResultFin::Success => TickResult::Success,
        }
    }
}
// It starts when the child is finished and the flow needs to go farther.
pub fn finalize(
    tpe: &FlowType,
    args: RtArgs,
    tick_args: RtArgs,
    res: TickResultFin,
    ctx: &mut TreeContext,
) -> RtResult<RNodeState> {
    match tpe {
        FlowType::Root => Ok(RNodeState::from(run_with(tick_args, 0, 1), res.into())),
        FlowType::Sequence | FlowType::RSequence => {
            let cursor = read_cursor(tick_args.clone())?;
            let len = read_len_or_zero(tick_args.clone());

            match res {
                TickResultFin::Failure(v) => {
                    let args =
                        run_with(tick_args.clone(), cursor, len).with(REASON, RtValue::str(v));

                    Ok(RNodeState::Failure(args))
                }
                TickResultFin::Success => {
                    if cursor == len - 1 {
                        Ok(RNodeState::Success(run_with(
                            tick_args.clone(),
                            cursor,
                            len,
                        )))
                    } else {
                        Ok(RNodeState::Running(run_with(
                            tick_args.clone(),
                            cursor + 1,
                            len,
                        )))
                    }
                }
            }
        }
        FlowType::MSequence => {
            let cursor = read_cursor(tick_args.clone())?;
            let len = read_len_or_zero(tick_args.clone());

            match res {
                TickResultFin::Failure(v) => {
                    let args = run_with(
                        tick_args.clone().with(P_CURSOR, RtValue::int(cursor)),
                        cursor,
                        len,
                    )
                    .with(REASON, RtValue::str(v));

                    Ok(RNodeState::Failure(args))
                }
                TickResultFin::Success => {
                    if cursor == len - 1 {
                        Ok(RNodeState::Success(run_with(
                            tick_args.clone(),
                            cursor,
                            len,
                        )))
                    } else {
                        Ok(RNodeState::Running(run_with(
                            tick_args.clone(),
                            cursor + 1,
                            len,
                        )))
                    }
                }
            }
        }

        FlowType::Fallback | FlowType::RFallback => {
            let cursor = read_cursor(tick_args.clone())?;
            let len = read_len_or_zero(tick_args.clone());

            match res {
                TickResultFin::Failure(v) => {
                    if cursor == len - 1 {
                        let args =
                            run_with(tick_args.clone(), cursor, len).with(REASON, RtValue::str(v));
                        Ok(RNodeState::Failure(args))
                    } else {
                        Ok(RNodeState::Running(run_with(
                            tick_args.clone(),
                            cursor + 1,
                            len,
                        )))
                    }
                }
                TickResultFin::Success => Ok(RNodeState::Success(run_with(
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
// it starts when the child returns running.
// This stage handles some peculiarities with the tearing state up and etc
pub fn monitor(
    tpe: &FlowType,
    args: RtArgs,
    tick_args: RtArgs,
    ctx: &mut TreeContext,
) -> RtResult<RNodeState> {
    match tpe {
        FlowType::Sequence => {
            let cursor = read_cursor(tick_args.clone())?;
            Ok(RNodeState::Running(
                tick_args.with(P_CURSOR, RtValue::int(cursor)),
            ))
        }
        FlowType::Fallback => {
            let cursor = read_cursor(tick_args.clone())?;
            Ok(RNodeState::Running(
                tick_args.with(P_CURSOR, RtValue::int(cursor)),
            ))
        }
        FlowType::MSequence => {
            let cursor = read_cursor(tick_args.clone())?;
            Ok(RNodeState::Running(
                tick_args.with(P_CURSOR, RtValue::int(cursor)),
            ))
        }
        _ => Ok(RNodeState::Running(tick_args)),
        // FlowType::Parallel => {}
    }
}
