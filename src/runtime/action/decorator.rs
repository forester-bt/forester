use crate::runtime::action::Tick;
use crate::runtime::args::{RtArgs, RtArgument, RtValue, RtValueNumber};
use crate::runtime::context::{RNodeState, TreeContext};
use crate::runtime::rtree::rnode::DecoratorType;
use crate::runtime::{RtOk, RtResult, RuntimeError, TickResult};
use std::thread::sleep;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

pub(crate) fn before(
    tpe: &DecoratorType,
    init_args: RtArgs,
    ctx: &mut TreeContext,
) -> RtResult<RNodeState> {
    match tpe {
        DecoratorType::Delay => {
            sleep(Duration::from_millis(
                u64::try_from(get_delay(init_args)?).unwrap(),
            ));
            Ok(RNodeState::run(1))
        }
        DecoratorType::Timeout => Ok(RNodeState::run_with(1, start_args())),
        _ => Ok(RNodeState::run(1)),
    }
}

pub(crate) fn during(
    tpe: &DecoratorType,
    init_args: RtArgs,
    run_args: RtArgs,
    ctx: &mut TreeContext,
) -> RtResult<RNodeState> {
    match tpe {
        DecoratorType::Timeout => {
            let timeout = init_args.first_as(RtValue::as_int).unwrap_or(0);

            let err = format!("the decorator timeout does not have a start time");
            let args = run_args.clone();
            let start = args
                .first_as(RtValue::as_int)
                .ok_or(RuntimeError::uex(err))?;
            let curr = get_ts();
            if curr - start >= timeout {
                Ok(RNodeState::fin(
                    TickResult::failure(format!("the timeout {timeout} exceeded")),
                    0,
                    1,
                ))
            } else {
                Ok(RNodeState::run_with(1, run_args))
            }
        }
        _ => Ok(RNodeState::run_with(1, run_args)),
    }
}

pub(crate) fn after(
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
        DecoratorType::ForceSuccess => match child_res {
            TickResult::Running => Ok(RNodeState::running(0, 1)),
            _ => Ok(RNodeState::fin(TickResult::success(), 0, 1)),
        },
        DecoratorType::ForceFail => match child_res {
            TickResult::Running => Ok(RNodeState::running(0, 1)),
            _ => Ok(RNodeState::fin(TickResult::failure_empty(), 0, 1)),
        },
        DecoratorType::Repeat => {
            let count = init_args.first_as(RtValue::as_int).unwrap_or(1);
            let attempt = run_args.first_as(RtValue::as_int).unwrap_or(1);

            if attempt >= count {
                Ok(RNodeState::fin(TickResult::success(), 0, 1))
            } else {
                let args = RtArgs(vec![RtArgument::new_noname(RtValue::Number(
                    RtValueNumber::Int(attempt + 1),
                ))]);
                Ok(RNodeState::run_with(1, args))
            }
        }
        DecoratorType::Timeout => match child_res {
            TickResult::Running => {
                let timeout = init_args.first_as(RtValue::as_int).unwrap_or(0);

                let err = format!("the decorator timeout does not have a start time");
                let args = run_args.clone();
                let start = args
                    .first_as(RtValue::as_int)
                    .ok_or(RuntimeError::uex(err))?;
                let curr = get_ts();
                if curr - start >= timeout {
                    Ok(RNodeState::fin(
                        TickResult::failure(format!("the timeout {timeout} exceeded")),
                        0,
                        1,
                    ))
                } else {
                    Ok(RNodeState::run_with(1, run_args))
                }
            }
            r => Ok(RNodeState::fin(r, 1, 1)),
        },
        _ => Err(RuntimeError::UnImplementedAction("decorator".to_string())),
        DecoratorType::Delay => match child_res {
            TickResult::Running => Ok(RNodeState::running(0, 1)),
            r => Ok(RNodeState::fin(r, 0, 1)),
        },
        DecoratorType::Retry => match child_res {
            TickResult::Success => Ok(RNodeState::fin(TickResult::success(), 1, 1)),
            TickResult::Failure(v) => {
                let count = init_args.first_as(RtValue::as_int).unwrap_or(0);
                let attempts = run_args.first_as(RtValue::as_int).unwrap_or(0);

                if attempts >= count {
                    Ok(RNodeState::fin(TickResult::failure(v), 0, 1))
                } else {
                    let args = RtArgs(vec![RtArgument::new_noname(RtValue::Number(
                        RtValueNumber::Int(attempts + 1),
                    ))]);
                    Ok(RNodeState::run_with(1, args))
                }
            }
            TickResult::Running => Ok(RNodeState::run_with(1, run_args)),
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
    let err = format!(
        "the decorator delay accepts one integer param, denoting duration of delay in millis"
    );
    args.first_as(RtValue::as_int).ok_or(RuntimeError::uex(err))
}
