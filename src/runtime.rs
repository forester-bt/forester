pub mod blackboard;
pub mod rtree;
pub mod action;
pub mod args;
pub mod rnode;
pub mod forester;
pub mod actions;

use std::collections::HashMap;
use serde::{Deserialize, Serialize};
use crate::tree::TreeError;



pub enum TickResult {
    Success,
    Failure,
    Running,
}

pub enum RuntimeError {
    UnImplementedAction(String),
    BlackBoardError(String)
}
