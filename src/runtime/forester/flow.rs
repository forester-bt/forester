use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::{RNodeState, TreeContext};
use crate::runtime::rtree::rnode::FlowType;
use crate::runtime::{RtResult, RuntimeError, TickResult};
use std::cmp::max;

// current child
pub const CURSOR: &str = "cursor";
// the child len
pub const LEN: &str = "len";

// the current cursor
// when the process is torn up(the child returns running or in seq is failure etc)
pub const P_CURSOR: &str = "prev_cursor";
// reason for the failure
pub const REASON: &str = "reason";

pub const CHILDREN_PAR: &str = "children";

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

// read and compare the cursor and prev_cursor
// if both are present, return the max
// if only one is present, return it
// if none is present, return 0
pub(crate) fn read_cursor(tick_args: RtArgs) -> RtResult<i64> {
    let p_cursor = tick_args.find(CURSOR.to_string()).and_then(RtValue::as_int);
    let cursor = tick_args
        .find(P_CURSOR.to_string())
        .and_then(RtValue::as_int);

    match (cursor, p_cursor) {
        (Some(lhs), Some(rhs)) => Ok(max(lhs, rhs)),
        (None, Some(v)) | (Some(v), None) => Ok(v),
        _ => Ok(0),
    }
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
    _args: RtArgs,
    tick_args: RtArgs,
    res: TickResultFin,
    _ctx: &mut TreeContext,
) -> RtResult<RNodeState> {
    match tpe {
        FlowType::Root => Ok(RNodeState::from(run_with(tick_args, 0, 1), res.into())),
        FlowType::Sequence | FlowType::RSequence => {
            let cursor = read_cursor(tick_args.clone())?;
            let len = read_len_or_zero(tick_args.clone());

            match res {
                TickResultFin::Failure(v) => {
                    let args = run_with(tick_args, cursor, len).with(REASON, RtValue::str(v));

                    Ok(RNodeState::Failure(args))
                }
                TickResultFin::Success => {
                    if cursor == len - 1 {
                        Ok(RNodeState::Success(run_with(tick_args, cursor, len)))
                    } else {
                        Ok(RNodeState::Running(run_with(tick_args, cursor + 1, len)))
                    }
                }
            }
        }
        FlowType::MSequence => {
            let cursor = read_cursor(tick_args.clone())?;
            let len = read_len_or_zero(tick_args.clone());

            match res {
                TickResultFin::Failure(v) => {
                    let args =
                        run_with(tick_args.with(P_CURSOR, RtValue::int(cursor)), cursor, len)
                            .with(REASON, RtValue::str(v));

                    Ok(RNodeState::Failure(args))
                }
                TickResultFin::Success => {
                    if cursor == len - 1 {
                        Ok(RNodeState::Success(run_with(tick_args, cursor, len)))
                    } else {
                        Ok(RNodeState::Running(run_with(tick_args, cursor + 1, len)))
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
                        let args = run_with(tick_args, cursor, len).with(REASON, RtValue::str(v));
                        Ok(RNodeState::Failure(args))
                    } else {
                        Ok(RNodeState::Running(run_with(tick_args, cursor + 1, len)))
                    }
                }
                TickResultFin::Success => Ok(RNodeState::Success(run_with(tick_args, cursor, len))),
            }
        }
        FlowType::Parallel => {
            let cursor = read_cursor(tick_args.clone())?;
            let len = read_len_or_zero(tick_args.clone());

            let st = match res {
                TickResultFin::Failure(_) => -1,
                TickResultFin::Success => 1
            };

            let tick_args = replace_child_state(tick_args, cursor as usize, st);

            if cursor == len - 1 {
                let mut elems = read_children_state(tick_args.clone());
                if elems.contains(&-1) {
                    let args = run_with(tick_args, cursor, len)
                        .with(REASON, RtValue::str("parallel failure".to_string()));
                    Ok(RNodeState::Failure(args))
                } else if elems.contains(&0) {
                    Ok(RNodeState::Running(run_with(tick_args, cursor, len)))
                } else {
                    Ok(RNodeState::Success(run_with(tick_args, cursor, len)))
                }
            } else {
                Ok(RNodeState::Running(run_with(tick_args, cursor + 1, len)))
            }
        }

        _ => Err(RuntimeError::UnImplementedAction("flow".to_string())),
    }
}

// it starts when the child returns running.
// This stage handles some peculiarities with the tearing state up and etc
pub fn monitor(
    tpe: &FlowType,
    _args: RtArgs,
    tick_args: RtArgs,
    _ctx: &mut TreeContext,
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
        FlowType::Parallel => {
            let cursor = read_cursor(tick_args.clone())?;
            let len = read_len_or_zero(tick_args.clone());

            let mut tick_args = tick_args;

            if tick_args.find(CHILDREN_PAR.to_string()).is_none() {
                tick_args = tick_args.with(
                    CHILDREN_PAR,
                    RtValue::Array(vec![RtValue::int(0); len as usize]),
                );
            }

            Ok(RNodeState::Running(
                replace_child_state(
                    tick_args.with(P_CURSOR, RtValue::int(cursor)),
                    cursor as usize, 0,
                )
            ))
        }
        _ => Ok(RNodeState::Running(tick_args)),
    }
}

fn replace_child_state(args: RtArgs, idx: usize, v: i64) -> RtArgs {
    let mut args = args;
    let mut elems = read_children_state(args.clone());

    elems.insert(idx, v);
    args.with(CHILDREN_PAR, RtValue::Array(elems.into_iter().map(RtValue::int).collect()))
}

fn read_children_state(args: RtArgs) -> Vec<i64> {
    args.find(CHILDREN_PAR.to_string())
        .and_then(|v| v.as_vec(|v| v.as_int().unwrap()))
        .unwrap_or_default()
}