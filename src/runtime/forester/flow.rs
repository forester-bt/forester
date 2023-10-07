use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::{RNodeState, TreeContext};
use crate::runtime::rtree::rnode::FlowType;
use crate::runtime::{RtResult, RuntimeError, TickResult};
use std::cmp::max;
use itertools::Itertools;
use FlowDecision::{PopNode, Stay};

// current child
pub const CURSOR: &str = "cursor";
// the child len
pub const LEN: &str = "len";

// the current cursor
// when the process is torn up(the child returns running or in seq is failure etc)
pub const P_CURSOR: &str = "prev_cursor";
// reason for the failure
pub const REASON: &str = "reason";

// the list of children and states, where
// 0 is ready,
// 1 is running,
// 2 is failure,
// 3 is success
pub const CHILDRENS: &str = "children";

pub fn run_with(tick_args: RtArgs, cursor: i64, len: i64) -> RtArgs {
    debug!(target:"params", "{}, cur:{cursor}, len:{len}", tick_args);
    tick_args
        .with(CURSOR, RtValue::int(cursor))
        .with(LEN, RtValue::int(len))
}

// parallel node needs to know the previous state of the children.
// It acts non reactively
// therefore if there is a previous state it tries to find a child that either running or ready
pub fn run_with_par(tick_args: RtArgs, cursor: i64, len: i64) -> RtArgs {
    let prev_children_states = read_children_state(tick_args.clone());
    if prev_children_states.is_empty() {
        run_with(
            tick_args.with(
                CHILDRENS,
                RtValue::Array(vec![RtValue::int(0); len as usize])),
            cursor, len)
    } else {
        let idx = prev_children_states
            .into_iter()
            .find_position(|c| *c == 0 || *c == 1).map(|(idx, _)| idx)
            .unwrap_or_default();
        run_with(tick_args, idx as i64, len)
    }
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
) -> RtResult<FlowDecision> {
    match tpe {
        FlowType::Root => Ok(Stay(RNodeState::from(run_with(tick_args, 0, 1), res.into()))),
        FlowType::Sequence | FlowType::RSequence => {
            let cursor = read_cursor(tick_args.clone())?;
            let len = read_len_or_zero(tick_args.clone());

            match res {
                TickResultFin::Failure(v) => {
                    let args = run_with(tick_args, cursor, len).with(REASON, RtValue::str(v));

                    Ok(Stay(RNodeState::Failure(args)))
                }
                TickResultFin::Success => {
                    if cursor == len - 1 {
                        Ok(Stay(RNodeState::Success(run_with(tick_args, cursor, len))))
                    } else {
                        Ok(Stay(RNodeState::Running(run_with(tick_args, cursor + 1, len))))
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

                    Ok(Stay(RNodeState::Failure(args)))
                }
                TickResultFin::Success => {
                    if cursor == len - 1 {
                        Ok(Stay(RNodeState::Success(run_with(tick_args, cursor, len))))
                    } else {
                        Ok(Stay(RNodeState::Running(run_with(tick_args, cursor + 1, len))))
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
                        Ok(Stay(RNodeState::Failure(run_with(tick_args, cursor, len)
                            .with(REASON, RtValue::str(v)))))
                    } else {
                        Ok(Stay(RNodeState::Running(run_with(tick_args, cursor + 1, len))))
                    }
                }
                TickResultFin::Success => Ok(Stay(RNodeState::Success(run_with(tick_args, cursor, len)))),
            }
        }
        FlowType::Parallel => {
            let cursor = read_cursor(tick_args.clone())?;
            let len = read_len_or_zero(tick_args.clone());

            let st = match res {
                TickResultFin::Failure(_) => 2,
                TickResultFin::Success => 3
            };

            let tick_args = replace_child_state(tick_args, cursor as usize, st);
            let children = read_children_state(tick_args.clone());

            // if some child is running or ready, we continue
            if let Some(idx) = find_next_idx(&children, cursor) {
                Ok(Stay(RNodeState::Running(
                    tick_args.with(CURSOR, RtValue::int(idx as i64)),
                )))
            } else {
                if children.contains(&1) || children.contains(&0) {
                    let next_cursor = find_first_idx(&children, cursor).unwrap_or(0);
                    let next_state = RNodeState::Running(
                        run_with(
                            tick_args,
                            next_cursor as i64, len)
                            .with(P_CURSOR, RtValue::int(0i64)) // reset the prev cursor
                    );
                    Ok(PopNode(next_state))
                } else if children.contains(&2) {
                    let args = run_with(tick_args, cursor, len)
                        .with(REASON, RtValue::str("parallel failure".to_string()));
                    // we stay allowing to remove us on the next it of the loop
                    Ok(Stay(RNodeState::Failure(args)))
                } else {
                    // we stay allowing to remove us on the next it of the loop
                    Ok(Stay(RNodeState::Success(run_with(tick_args, cursor, len))))
                }
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
) -> RtResult<FlowDecision> {
    match tpe {
        FlowType::Sequence => {
            let cursor = read_cursor(tick_args.clone())?;
            Ok(PopNode(RNodeState::Running(
                tick_args.with(P_CURSOR, RtValue::int(cursor)),
            )))
        }
        FlowType::Fallback => {
            let cursor = read_cursor(tick_args.clone())?;
            Ok(PopNode(RNodeState::Running(
                tick_args.with(P_CURSOR, RtValue::int(cursor)),
            )))
        }
        FlowType::MSequence => {
            let cursor = read_cursor(tick_args.clone())?;
            Ok(PopNode(RNodeState::Running(
                tick_args.with(P_CURSOR, RtValue::int(cursor)),
            )))
        }
        FlowType::Parallel => {
            let mut cursor = read_cursor(tick_args.clone())?;
            let new_args = replace_child_state(
                tick_args.with(P_CURSOR, RtValue::int(cursor)),
                cursor as usize,
                1,
            );

            let children = read_children_state(new_args.clone());

            if let Some(idx) = find_next_idx(&children, cursor) {
                Ok(Stay(RNodeState::Running(
                    new_args.with(CURSOR, RtValue::int(idx as i64)),
                )))
            } else {
                Ok(PopNode(RNodeState::Running(new_args)))
            }
        }
        _ => Ok(PopNode(RNodeState::Running(tick_args))),
    }
}

// decision impacts on the case when we decide if we stay on the node
// and go farther down or we climb up the tree
// basically it processes the case when we have a running child before and after cursor.
// in the latter we stay and in the former we climb up and eventually end up ticking the root
#[derive(Debug, Clone)]
pub enum FlowDecision {
    PopNode(RNodeState),
    Stay(RNodeState),
}

fn replace_child_state(args: RtArgs, idx: usize, v: i64) -> RtArgs {
    let mut args = args;
    let mut elems = read_children_state(args.clone());
    debug!(target:"params in child", "prev : [{args}], idx:{idx}, new state: {v}");
    elems[idx] = v;
    args.with(CHILDRENS, RtValue::Array(elems.into_iter().map(RtValue::int).collect()))
}

fn read_children_state(args: RtArgs) -> Vec<i64> {
    args.find(CHILDRENS.to_string())
        .and_then(|v| v.as_vec(|v| v.as_int().unwrap()))
        .unwrap_or_default()
}

// find the next idx that is either running or ready
fn find_next_idx(children: &Vec<i64>, current: i64) -> Option<usize> {
    let mut cursor = (current + 1) as usize;
    let mut next_idx = None;
    while cursor < children.len() {
        if children[cursor] == 0 || children[cursor] == 1 {
            next_idx = Some(cursor);
            break;
        }
        cursor = cursor + 1;
    }
    next_idx
}

// find the next idx that is either running or ready before the current idx
fn find_first_idx(children: &Vec<i64>, current: i64) -> Option<usize> {
    let mut cursor = 0;
    let mut next_idx = None;
    while cursor < current as usize {
        if children[cursor] == 0 || children[cursor] == 1 {
            next_idx = Some(cursor);
            break;
        }
        cursor = cursor + 1;
    }
    next_idx
}