use crate::runtime::args::{RtArgs, RtArgument, RtValue, RtValueNumber};
use crate::runtime::context::{RNodeState, TreeContext};
use crate::runtime::forester::flow::{run_with, LEN, REASON};
use crate::runtime::rtree::rnode::DecoratorType;
use crate::runtime::{RtResult, RuntimeError, TickResult};
use std::thread::sleep;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

pub const ATTEMPT: &str = "attempt";
pub const TIMESTAMP: &str = "timestamp";

// It runs on the preparation stage when the child is ready but not running.
// It is useful to save some information before(counters, timeout etc)
pub(crate) fn prepare(
    tpe: &DecoratorType,
    init_args: RtArgs,
    tick_args: RtArgs,
    ctx: &mut TreeContext,
) -> RtResult<RNodeState> {
    debug!(target:"decorator[prepare]", "tick:{}, type:{} where decorator args({init_args}) and child args({tick_args})",ctx.curr_ts(), tpe);
    match tpe {
        DecoratorType::Delay => {
            sleep(Duration::from_millis(
                u64::try_from(get_delay(init_args)?).unwrap(),
            ));
            Ok(RNodeState::Running(run_with(tick_args, 0, 1)))
        }
        DecoratorType::Timeout => {
            let args = tick_args.with(
                TIMESTAMP,
                RtValue::Number(RtValueNumber::Int(get_system_timestamp())),
            );
            Ok(RNodeState::Running(run_with(args, 0, 1)))
        }
        DecoratorType::Repeat | DecoratorType::Retry => {
            let args = tick_args.with(ATTEMPT, RtValue::Number(RtValueNumber::Int(1)));
            Ok(RNodeState::Running(run_with(args, 0, 1)))
        }
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
    debug!(target:"decorator[monitor]", "tick:{}, type:{}",ctx.curr_ts(), tpe);
    match tpe {
        DecoratorType::Timeout => {
            let timeout_period = init_args.first_as(RtValue::as_int).unwrap_or(0);

            let err = "the decorator timeout does not have a start time".to_string();
            let args = tick_args.clone();
            let start_timestamp = args
                .find_as(TIMESTAMP.to_string(), RtValue::as_int)
                .ok_or(RuntimeError::fail(err))?;

            let current_timestamp = get_system_timestamp();
            if current_timestamp - start_timestamp >= timeout_period {
                let args = tick_args.with(
                    REASON,
                    RtValue::str(format!(
                        "the timeout of {timeout_period} milliseconds exceeded"
                    )),
                );

                Ok(RNodeState::Failure(run_with(args, 0, 1)))
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
    debug!(target:"decorator[fin]", "tick:{}, type:{}",ctx.curr_ts(), tpe);
    match tpe {
        DecoratorType::Inverter => match child_res {
            TickResult::Success => {
                let args = run_with(tick_args, 1, 1).with(
                    REASON,
                    RtValue::str("decorator inverts the result.".to_string()),
                );
                Ok(RNodeState::Failure(args))
            }
            _ => Ok(RNodeState::Success(run_with(tick_args, 0, 1))),
        },
        DecoratorType::ForceSuccess => Ok(RNodeState::Success(run_with(tick_args, 0, 1))),
        DecoratorType::ForceFail => Ok(RNodeState::Failure(
            run_with(tick_args, 0, 1)
                .with(REASON, RtValue::str("decorator force fail.".to_string())),
        )),
        DecoratorType::Repeat => match child_res {
            TickResult::Success => {
                let max_attempts = init_args.first_as(RtValue::as_int).unwrap_or(0);

                let err = "the decorator repeat does not have an attempt count".to_string();
                let args = tick_args.clone();
                let attempt = args
                    .find_as(ATTEMPT.to_string(), RtValue::as_int)
                    .ok_or(RuntimeError::fail(err))?;

                if max_attempts > 0 && attempt >= max_attempts {
                    Ok(RNodeState::Success(run_with(tick_args, 0, 1)))
                } else {
                    let args =
                        tick_args.with(ATTEMPT, RtValue::Number(RtValueNumber::Int(attempt + 1)));
                    Ok(RNodeState::Running(run_with(args, 0, 1)))
                }
            }
            TickResult::Failure(v) => {
                let args = tick_args.with(REASON, RtValue::str(v));
                Ok(RNodeState::Failure(run_with(args, 0, 1)))
            }
            TickResult::Running => Ok(RNodeState::Running(run_with(tick_args, 0, 1))),
        },
        DecoratorType::Timeout => Ok(RNodeState::from(run_with(tick_args, 1, 1), child_res)),
        DecoratorType::Delay => Ok(RNodeState::from(run_with(tick_args, 0, 1), child_res)),
        DecoratorType::Retry => match child_res {
            TickResult::Failure(v) => {
                let max_attempts = init_args.first_as(RtValue::as_int).unwrap_or(0);

                let err = "the decorator retry does not have an attempt count".to_string();
                let args = tick_args.clone();
                let attempt = args
                    .find_as(ATTEMPT.to_string(), RtValue::as_int)
                    .ok_or(RuntimeError::fail(err))?;

                if max_attempts > 0 && attempt >= max_attempts {
                    let args = tick_args.with(REASON, RtValue::str(v));
                    Ok(RNodeState::Failure(run_with(args, 0, 1)))
                } else {
                    let args =
                        tick_args.with(ATTEMPT, RtValue::Number(RtValueNumber::Int(attempt + 1)));
                    Ok(RNodeState::Running(run_with(args, 0, 1)))
                }
            }
            TickResult::Success => Ok(RNodeState::Success(run_with(tick_args, 0, 1))),
            _ => Ok(RNodeState::Running(run_with(tick_args, 0, 1))),
        },
    }
}

fn get_system_timestamp() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("should be able to get the system time since UNIX_EPOCH")
        .as_millis() as i64
}

fn get_delay(args: RtArgs) -> RtResult<i64> {
    let err = "the decorator delay accepts one integer param, denoting duration of delay in millis"
        .to_string();
    args.first_as(RtValue::as_int)
        .ok_or(RuntimeError::fail(err))
}
