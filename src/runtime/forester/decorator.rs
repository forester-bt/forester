use crate::runtime::args::{RtArgs, RtArgument, RtValue, RtValueNumber};
use crate::runtime::context::{RNodeState, TreeContext};
use crate::runtime::forester::flow::{run_with, LEN, REASON};
use crate::runtime::rtree::rnode::DecoratorType;
use crate::runtime::{RtResult, RuntimeError, TickResult};
use std::thread::sleep;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

// It runs on the preparation stage when the child is ready but not running.
// It is useful to save some information before(counters, timeout etc)
pub(crate) fn prepare(
    tpe: &DecoratorType,
    init_args: RtArgs,
    tick_args: RtArgs,
    ctx: &mut TreeContext,
) -> RtResult<RNodeState> {
    debug!(target:"> decorator::prepare", "tick:{}, type:{}",ctx.curr_ts(), tpe);
    match tpe {
        DecoratorType::Delay => {
            sleep(Duration::from_millis(
                u64::try_from(get_delay(init_args)?).unwrap(),
            ));
            Ok(RNodeState::Running(run_with(tick_args, 0, 1)))
        }
        DecoratorType::Timeout => Ok(RNodeState::Running(start_args().with(LEN, RtValue::int(1)))),
        _ => Ok(RNodeState::Running(tick_args.with(LEN, RtValue::int(1)))),
    }
}
// This runs when the child returns running.
// It works for timeout and other controlling decorators
pub(crate) fn monitor(
    tpe: &DecoratorType,
    init_args: RtArgs,
    tick_args: RtArgs,
    ctx: &mut TreeContext,
) -> RtResult<RNodeState> {
    debug!(target:"> decorator::monitor", "tick:{}, type:{}",ctx.curr_ts(), tpe);
    match tpe {
        DecoratorType::Timeout => {
            let timeout = init_args.first_as(RtValue::as_int).unwrap_or(0);
            let err = "the decorator timeout does not have a start time".to_string();
            let args = tick_args.clone();
            let start = args
                .first_as(RtValue::as_int)
                .ok_or(RuntimeError::fail(err))?;
            let curr = get_ts();
            if curr - start >= timeout {
                let args = run_with(tick_args, 0, 1).with(
                    REASON,
                    RtValue::str(format!("the timeout {timeout} exceeded")),
                );

                Ok(RNodeState::Failure(args))
            } else {
                Ok(RNodeState::Running(tick_args.with(LEN, RtValue::int(1))))
            }
        }
        _ => Ok(RNodeState::Running(tick_args.with(LEN, RtValue::int(1)))),
    }
}

// It works when the child is finished to process the result and pass it farther
pub(crate) fn finalize(
    tpe: &DecoratorType,
    tick_args: RtArgs,
    init_args: RtArgs,
    child_res: TickResult,
    ctx: &mut TreeContext,
) -> RtResult<RNodeState> {
    debug!(target:"> decorator::fin", "tick:{}, type:{}",ctx.curr_ts(), tpe);
    match tpe {
        DecoratorType::Inverter => match child_res {
            TickResult::Success => {
                let args = run_with(tick_args, 1, 1).with(
                    REASON,
                    RtValue::str("decorator inverts the result.".to_string()),
                );
                Ok(RNodeState::Failure(args))
            }
            TickResult::Failure(_) => Ok(RNodeState::Success(run_with(tick_args, 0, 1))),
            TickResult::Running => Ok(RNodeState::Running(run_with(tick_args, 0, 1))),
        },
        DecoratorType::ForceSuccess => match child_res {
            TickResult::Running => Ok(RNodeState::Running(run_with(tick_args, 0, 1))),
            _ => Ok(RNodeState::Success(run_with(tick_args, 0, 1))),
        },
        DecoratorType::ForceFail => match child_res {
            TickResult::Running => Ok(RNodeState::Running(run_with(tick_args, 0, 1))),
            _ => Ok(RNodeState::Failure(run_with(tick_args, 0, 1).with(
                REASON,
                RtValue::str("decorator force fail.".to_string()),
            ))),
        },
        DecoratorType::Repeat => {
            let count = init_args.first_as(RtValue::as_int).unwrap_or(1);
            let attempt = tick_args.first_as(RtValue::as_int).unwrap_or(1);
            if attempt >= count {
                Ok(RNodeState::Success(run_with(tick_args, 0, 1)))
            } else {
                let args = RtArgs(vec![RtArgument::new_noname(RtValue::int(attempt + 1))]);
                Ok(RNodeState::Running(run_with(args, 0, 1)))
            }
        }
        DecoratorType::Timeout => match child_res {
            TickResult::Running => {
                let timeout = init_args.first_as(RtValue::as_int).unwrap_or(0);
                let err = "the decorator timeout does not have a start time".to_string();
                let args = tick_args.clone();
                let start = args
                    .first_as(RtValue::as_int)
                    .ok_or(RuntimeError::fail(err))?;
                let curr = get_ts();
                if curr - start >= timeout {
                    let args = run_with(tick_args, 0, 1)
                        .with(REASON, RtValue::str("decorator force fail.".to_string()));
                    Ok(RNodeState::Failure(args))
                } else {
                    Ok(RNodeState::Running(run_with(tick_args, 0, 1)))
                }
            }
            r => Ok(RNodeState::from(run_with(tick_args, 1, 1), r)),
        },
        DecoratorType::Delay => match child_res {
            TickResult::Running => Ok(RNodeState::Running(run_with(tick_args, 0, 1))),
            r => Ok(RNodeState::from(run_with(tick_args, 0, 1), r)),
        },
        DecoratorType::Retry => match child_res {
            TickResult::Success => Ok(RNodeState::Success(run_with(tick_args, 1, 1))),
            TickResult::Failure(v) => {
                let count = init_args.first_as(RtValue::as_int).unwrap_or(0);
                let attempts = tick_args.first_as(RtValue::as_int).unwrap_or(0);

                if attempts >= count {
                    let args = run_with(tick_args, 0, 1).with(REASON, RtValue::str(v));
                    Ok(RNodeState::Failure(args))
                } else {
                    let args = RtArgs(vec![RtArgument::new_noname(RtValue::int(attempts + 1))]);
                    Ok(RNodeState::Running(run_with(args, 0, 1)))
                }
            }
            TickResult::Running => Ok(RNodeState::Running(run_with(tick_args, 0, 1))),
        },
    }
}

fn get_ts() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("")
        .as_secs() as i64
}
fn start_args() -> RtArgs {
    RtArgs(vec![RtArgument::new_noname(RtValue::Number(
        RtValueNumber::Int(get_ts()),
    ))])
}
fn get_delay(args: RtArgs) -> RtResult<i64> {
    let err = "the decorator delay accepts one integer param, denoting duration of delay in millis"
        .to_string();
    args.first_as(RtValue::as_int)
        .ok_or(RuntimeError::fail(err))
}
