use crate::runtime::action::builtin::data::{CheckEq, LockUnlockBBKey, Locked, StoreData, StoreTick, TestBool, Less};
use crate::runtime::action::builtin::http::HttpGet;
use crate::runtime::action::builtin::ReturnResult;
use crate::runtime::action::{Action, ActionName};
use crate::runtime::{RtResult, RuntimeError};
use crate::runtime::action::builtin::daemon::{CheckDaemonAction, StopDaemonAction};
use crate::runtime::builder::{ros_core, ros_nav};
use crate::tree::project::FileName;


pub(super) fn pick_action(action: &ActionName, file: &FileName) -> RtResult<Action> {

    match file.as_str() {
        "std::actions" => action_impl(action),
        "ros::nav2" => ros_nav::action_impl(action),
        "ros::core" => ros_core::action_impl(action),
        _ => Err(RuntimeError::UnImplementedAction(format!("{}::{}", file, action)))
    }
}

/// Built-in actions
/// The actions are accessible using the import 'import "std::actions"'

fn action_impl(action: &ActionName) -> RtResult<Action> {
    match action.as_str() {
        "fail_empty" => Ok(Action::sync(ReturnResult::fail_empty())),
        "fail" => Ok(Action::sync(ReturnResult::fail_empty())),
        "success" => Ok(Action::sync(ReturnResult::success())),
        "running" => Ok(Action::sync(ReturnResult::running())),
        "store" => Ok(Action::sync(StoreData)),
        "equal" => Ok(Action::sync(CheckEq)),
        "less" => Ok(Action::sync(Less)),
        "test" => Ok(Action::sync(TestBool)),
        "store_tick" => Ok(Action::sync(StoreTick)),
        "http_get" => Ok(Action::sync(HttpGet)),
        "http_get_async" => Ok(Action::a_sync(HttpGet)),
        "lock" => Ok(Action::sync(LockUnlockBBKey::Lock)),
        "unlock" => Ok(Action::sync(LockUnlockBBKey::Unlock)),
        "locked" => Ok(Action::sync(Locked)),
        "stop_daemon" => Ok(Action::sync(StopDaemonAction)),
        "daemon_alive" => Ok(Action::sync(CheckDaemonAction)),
        _ => Err(RuntimeError::UnImplementedAction(format!("std::actions::{}", action))),
    }
}


pub fn builtin_actions_file() -> String {
    r#"
//
// Built-in actions.
// The actions are accessible using the import 'import "std::actions"'
// Better to avoid modifying the file
//

// Fails execution, returning Result::Failure
impl fail(reason:string);
impl fail_empty();

// Success execution, returning Result::Success
impl success();

// Running execution, returning Result::Running
impl running();

// Sleeps on duration(milliseconds) then returns Result::Success
// impl sleep(duration:num);

// Stores the value in the given key. Returns Result::Success.
// If the cell is locked, returns Result::Failure
impl store(key:string, value:any);

// Compares a given value with what is in the cell:
// - Returns Result::Success if they are equal
// - Returns Fail(reason)if they are not equal
// - Returns Fail(reason) if there is no cell in bbe with the given key.
impl equal(key:string, expected:any);

// Compares a given value with what is in the cell:
// - Returns Result::Success if lhs is less then rhs
// - Returns Fail(reason)if otherwise
// - Returns Fail(reason) if there is no cell in bbe with the given key.
impl less(lhs:num, rhs:num);

// Compares given bool value with true:
// - Returns Result::Success if they are equal
// - Returns Fail(reason)if they are not equal
// - Returns Fail(reason) if there is no cell in bbe with the given key.
impl test(key:string);

/// Store the current tick
impl store_tick(name:string);

/// Performs http get request
impl http_get(url:string, bb_key:string);

/// Performs http get request
impl http_get_async(url:string, bb_key:string);

// Lock key in bb
impl lock(key:string);

// Unlock key in bb
impl unlock(key:string);

// Validate the key if it is locked in bb
impl locked(key:string);

// Stop the daemon by name
// if there is no daemon the action returns Result::Success
// otherwise the result of the action(likely success)
impl stop_daemon(name:string);

// Check if the daemon is running
// if there is no daemon the action returns Result::Failure otherwise Result::Success
impl daemon_alive(name:string);

"#
        .to_string()
}