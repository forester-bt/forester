use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::{RNodeState, TreeContext};
use crate::runtime::rtree::rnode::FlowType;
use crate::runtime::{RtResult, RuntimeError, TickResult};
use std::cmp::max;
use FlowDecision::{Halt, PopNode, Stay};

type HaltingChildCursor = usize;

// current child
pub const CURSOR: &str = "cursor";
// the child len
pub const LEN: &str = "len";

// the current cursor
// when the process is torn up(the child returns running or in seq is failure etc)
pub const P_CURSOR: &str = "prev_cursor";
// reason for the failure
pub const REASON: &str = "reason";
// Record of the currently running child, in case a reactive flow node needs to halt it.
// This typically stores the same information as P_CURSOR, but doesn't affect read_cursor() results.
pub const RUNNING_CHILD: &str = "running_child";

// the list of children and states, where
// 0 is ready,
// 1 is running,
// 2 is failure,
// 3 is success
pub const CHILDREN: &str = "children";

pub fn run_with(tick_args: RtArgs, cursor: i64, len: i64) -> RtArgs {
    debug!(target:"params", "{}, cur:{cursor}, len:{len}", tick_args);
    tick_args
        .with(CURSOR, RtValue::int(cursor))
        .with(LEN, RtValue::int(len))
}

// parallel node needs to know the previous state of the children.
// It acts non reactively
// therefore if there is a previous state it tries to find a child that either running or ready
pub fn run_with_par(tick_args: RtArgs, len: i64) -> RtArgs {
    let prev_states = read_children_state(tick_args.clone());
    if prev_states.is_empty() {
        // the first time we create the children array
        run_with(
            tick_args.with(
                CHILDREN,
                RtValue::Array(vec![RtValue::int(0); len as usize]),
            ),
            0,
            len,
        )
    } else {
        run_with(tick_args.clone(), read_cursor(tick_args).unwrap_or(0), len)
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
        FlowType::Root => Ok(Stay(RNodeState::from(
            run_with(tick_args, 0, 1),
            res.into(),
        ))),
        FlowType::Sequence | FlowType::RSequence => {
            let cursor = read_cursor(tick_args.clone())?;
            let len = read_len_or_zero(tick_args.clone());
            let running_child = tick_args
                .find(RUNNING_CHILD.to_string())
                .and_then(RtValue::as_int);

            // There's only one scenario where we don't remove RUNNING_CHILD, we'll re-add it if that's the case.
            let mut args = tick_args.remove(RUNNING_CHILD).remove(P_CURSOR);

            match res {
                TickResultFin::Failure(v) => {
                    let args = args.with(REASON, RtValue::str(v));

                    // Failure will interrupt a reactive sequence, check if we need to halt a running child
                    if let Some(running) = running_child {
                        if running > cursor {
                            // This failure result needs to interrupt the running child.
                            // Note non-reactive sequences will always have running == p_cursor == cursor, so this will be unreachable for them.
                            return Ok(Halt(
                                RNodeState::Failure(run_with(args, cursor, len)),
                                running as usize,
                            ));
                        }
                    }

                    Ok(Stay(RNodeState::Failure(run_with(args, cursor, len))))
                }
                TickResultFin::Success => {
                    if cursor == len - 1 {
                        Ok(Stay(RNodeState::Success(run_with(args, cursor, len))))
                    } else {
                        if let Some(running) = running_child {
                            if running > cursor {
                                // We haven't reached the previously running child yet, re-add it.
                                args = args.with(RUNNING_CHILD, RtValue::int(running))
                            }
                        }
                        Ok(Stay(RNodeState::Running(run_with(args, cursor + 1, len))))
                    }
                }
            }
        }
        FlowType::MSequence => {
            let cursor = read_cursor(tick_args.clone())?;
            let len = read_len_or_zero(tick_args.clone());
            let running_child = tick_args
                .find(RUNNING_CHILD.to_string())
                .and_then(RtValue::as_int);

            // There's only one scenario where we don't remove RUNNING_CHILD, we'll re-add it if that's the case.
            let mut args = tick_args.remove(RUNNING_CHILD);

            match res {
                TickResultFin::Failure(v) => {
                    let args = run_with(args.with(P_CURSOR, RtValue::int(cursor)), cursor, len)
                        .with(REASON, RtValue::str(v));

                    Ok(Stay(RNodeState::Failure(args)))
                }
                TickResultFin::Success => {
                    if cursor == len - 1 {
                        // Remove P_CURSOR so that the next tick will start from the beginning
                        let args = args.remove(P_CURSOR);
                        Ok(Stay(RNodeState::Success(run_with(args, cursor, len))))
                    } else {
                        if let Some(running) = running_child {
                            if running > cursor {
                                // We haven't reached the previously running child yet, re-add it.
                                args = args.with(RUNNING_CHILD, RtValue::int(running))
                            }
                        }
                        Ok(Stay(RNodeState::Running(run_with(args, cursor + 1, len))))
                    }
                }
            }
        }

        FlowType::Fallback | FlowType::RFallback => {
            let cursor = read_cursor(tick_args.clone())?;
            let len = read_len_or_zero(tick_args.clone());
            let running_child = tick_args
                .find(RUNNING_CHILD.to_string())
                .and_then(RtValue::as_int);

            // There's only one scenario where we don't remove RUNNING_CHILD, we'll re-add it if that's the case.
            let mut args = tick_args.remove(RUNNING_CHILD).remove(P_CURSOR);

            match res {
                TickResultFin::Failure(v) => {
                    if cursor == len - 1 {
                        let args = args.with(REASON, RtValue::str(v));
                        Ok(Stay(RNodeState::Failure(run_with(args, cursor, len))))
                    } else {
                        if let Some(running) = running_child {
                            if running > cursor {
                                // We haven't reached the previously running child yet, re-add it.
                                args = args.with(RUNNING_CHILD, RtValue::int(running))
                            }
                        }
                        Ok(Stay(RNodeState::Running(run_with(args, cursor + 1, len))))
                    }
                }
                TickResultFin::Success => {
                    // Success will interrupt a reactive fallback, check if we need to halt a running child
                    if let Some(running) = running_child {
                        if running > cursor {
                            // This success result needs to interrupt the running child.
                            // Note non-reactive fallbacks will always have running == p_cursor == cursor, so this will be unreachable for them.
                            return Ok(Halt(
                                RNodeState::Success(run_with(args, cursor, len)),
                                running as usize,
                            ));
                        }
                    }

                    // This success result is just like any other fallback success
                    Ok(Stay(RNodeState::Success(run_with(args, cursor, len))))
                }
            }
        }
        FlowType::Parallel => {
            let cursor = read_cursor(tick_args.clone())?;
            let len = read_len_or_zero(tick_args.clone());
            let st = match res {
                TickResultFin::Failure(_) => 2,
                TickResultFin::Success => 3,
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
                        run_with(tick_args, next_cursor as i64, len)
                            // reset the prev cursor otherwise it weill be greater then the current cursor and the prev one will be taken
                            .with(P_CURSOR, RtValue::int(0i64)),
                    );
                    // we know there are some nodes needs to be run but they are behind so we can touch them in the next tick only.
                    // And we pop up the node to allow the next tick to run the children
                    // if we stay the running nodes will be touched in the same tick
                    Ok(PopNode(next_state))
                } else if children.contains(&2) {
                    let args = run_with(tick_args, cursor, len)
                        .with(REASON, RtValue::str("parallel failure".to_string()))
                        .remove(CHILDREN);
                    // we stay allowing to remove us on the next iteration of the loop
                    Ok(Stay(RNodeState::Failure(args)))
                } else {
                    // we stay allowing to remove us on the next iteration of the loop
                    Ok(Stay(RNodeState::Success(
                        run_with(tick_args, cursor, len).remove(CHILDREN),
                    )))
                }
            }
        }
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
        FlowType::RSequence | FlowType::RFallback => {
            // RSequence and RFallback don't use P_CURSOR
            // let's get the cursor manually so P_CURSOR doesn't accidentially poison our result.
            let cursor = tick_args
                .find(CURSOR.to_string())
                .and_then(RtValue::as_int)
                .unwrap_or(0);
            let previous_running_child = tick_args
                .find(RUNNING_CHILD.to_string())
                .and_then(RtValue::as_int);

            let new_state =
                RNodeState::Running(tick_args.with(RUNNING_CHILD, RtValue::int(cursor)));

            // Check there isn't another child already being tracked by the RUNNING_CHILD key.
            // If there is we'll need to halt that before going any further.
            if let Some(prev_running_child_cursor) = previous_running_child {
                if prev_running_child_cursor > cursor {
                    return Ok(Halt(new_state, prev_running_child_cursor as usize));
                }
            }

            // There was no other previously running child, continue as normal
            Ok(PopNode(new_state))
        }
        FlowType::Sequence | FlowType::MSequence | FlowType::Fallback => {
            let cursor = read_cursor(tick_args.clone())?;
            Ok(PopNode(RNodeState::Running(
                tick_args
                    .with(RUNNING_CHILD, RtValue::int(cursor))
                    .with(P_CURSOR, RtValue::int(cursor)),
            )))
        }
        FlowType::Parallel => {
            let cursor = read_cursor(tick_args.clone())?;
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

// Handle ticking a flow node with the state "Halting".
// Returns a tuple of the new state and the cursor position of the child to be halted, if one exists.
pub fn halt(flow_type: &FlowType, tick_args: RtArgs) -> (RNodeState, Option<usize>) {
    match flow_type {
        FlowType::Sequence
        | FlowType::MSequence
        | FlowType::RSequence
        | FlowType::Fallback
        | FlowType::RFallback => {
            // Sequence/fallback nodes check if they need to halt a child at the RUNNING_CHILD cursor position.
            let running_child_cursor = tick_args
                .find(RUNNING_CHILD.to_string())
                .and_then(RtValue::as_int)
                .map(|v| v as usize);
            let mut args = tick_args.remove(RUNNING_CHILD);

            // MSequence needs to keep its position, but other nodes don't.
            if flow_type != &FlowType::MSequence {
                args = args.remove(P_CURSOR);
            }

            let new_state = RNodeState::Ready(args);
            (new_state, running_child_cursor)
        }
        _ => (RNodeState::Ready(tick_args), None),
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
    Halt(RNodeState, HaltingChildCursor),
}

fn replace_child_state(args: RtArgs, idx: usize, v: i64) -> RtArgs {
    let args = args;
    let mut elems = read_children_state(args.clone());
    debug!(target:"params in child", "prev : [{args}], idx:{idx}, new state: {v}");
    elems[idx] = v;
    args.with(
        CHILDREN,
        RtValue::Array(elems.into_iter().map(RtValue::int).collect()),
    )
}

fn read_children_state(args: RtArgs) -> Vec<i64> {
    args.find(CHILDREN.to_string())
        .and_then(|v| v.as_vec(|v| v.as_int().unwrap()))
        .unwrap_or_default()
}

// find the next idx that is either running or ready
fn find_next_idx(children: &Vec<i64>, current: i64) -> Option<usize> {
    find_pos(children, current + 1, children.len() as i64)
}

// find the next idx that is either running or ready before the current idx
fn find_first_idx(children: &Vec<i64>, current: i64) -> Option<usize> {
    find_pos(children, 0, current)
}

fn find_pos(children: &Vec<i64>, low: i64, high: i64) -> Option<usize> {
    let mut cursor = low as usize;
    let mut next_idx = None;
    while cursor < high as usize {
        if children[cursor] == 0 || children[cursor] == 1 {
            next_idx = Some(cursor);
            break;
        }
        cursor = cursor + 1;
    }
    next_idx
}
