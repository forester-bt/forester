pub mod action;
pub mod args;
pub mod blackboard;
pub mod forester;
pub mod rtree;

use crate::tree::TreeError;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

pub type RtResult<T> = Result<T, RuntimeErrorCause>;

pub enum TickResult {
    Success,
    Failure,
    Running,
}
#[derive(Debug)]
pub enum RuntimeErrorCause {
    UnImplementedAction(String),
    BlackBoardError(String),
    IOError(String),
    Unexpected(String),
    WrongArgument(String),
}
