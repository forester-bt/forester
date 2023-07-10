pub mod action;
pub mod args;
pub mod blackboard;
pub mod builder;
pub mod context;
pub mod forester;
pub mod rtree;

use crate::runtime::action::Tick;
use crate::tree::TreeError;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

pub type RtResult<T> = Result<T, RuntimeError>;
pub type RtOk = Result<(), RuntimeError>;
#[derive(Clone, Debug, PartialEq)]
pub enum TickResult {
    Success,
    Failure(String),
    Running,
}

impl TickResult {
    pub fn success() -> TickResult {
        TickResult::Success
    }
    pub fn failure_empty() -> TickResult {
        TickResult::Failure("".to_string())
    }
    pub fn failure(reason: String) -> TickResult {
        TickResult::Failure(reason)
    }
    pub fn running() -> TickResult {
        TickResult::Running
    }
}

#[derive(Debug, PartialEq)]
pub enum RuntimeError {
    CompileError(TreeError),
    UnImplementedAction(String),
    BlackBoardError(String),
    IOError(String),
    Unexpected(String),
    WrongArgument(String),
    Stopped(String),
}

impl RuntimeError {
    pub fn uex(s: String) -> Self {
        Self::Unexpected(s)
    }
    pub fn bb(s: String) -> Self {
        Self::BlackBoardError(s)
    }
}

impl From<TreeError> for RuntimeError {
    fn from(value: TreeError) -> Self {
        RuntimeError::CompileError(value)
    }
}
impl From<serde_yaml::Error> for RuntimeError {
    fn from(value: serde_yaml::Error) -> Self {
        RuntimeError::IOError(value.to_string())
    }
}
impl From<serde_json::Error> for RuntimeError {
    fn from(value: serde_json::Error) -> Self {
        RuntimeError::IOError(value.to_string())
    }
}
impl From<std::io::Error> for RuntimeError {
    fn from(value: std::io::Error) -> Self {
        RuntimeError::IOError(value.to_string())
    }
}
