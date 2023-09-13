//! Forester represents a framework that provides the set of tools to perform the effective orchestration of the set of tasks.
//! The tasks can be performed synchronously or asynchronously, locally or remotely.
//! Forester takes care of the correct performance and distribution of the tasks.
//! the main concept of the framework is the flow based on the behavior trees
//! it can be effectively used in the game, ai, robotic areas, or anywhere where the workflow engine can be applied.
#[macro_use]
extern crate log;

use crate::runtime::RtResult;
use std::fs;
use std::path::PathBuf;

pub mod runtime;
pub mod simulator;
pub mod tracer;
pub mod tree;
pub mod visualizer;
pub mod exporter;

use crate::runtime::RuntimeError;
#[cfg(test)]
mod tests;

pub fn read_file(pb: &PathBuf) -> RtResult<String> {
    fs::read_to_string(pb)
        .map_err(|e| RuntimeError::IOError(format!("error:{e}, file:{}", pb.as_path().display())))
}

pub(crate) fn get_pb(file_pb: &PathBuf, root: &Option<PathBuf>) -> RtResult<PathBuf> {
    if file_pb.is_relative() {
        match root {
            Some(r) => {
                let mut full_path = r.clone();
                full_path.push(file_pb);
                Ok(full_path)
            }
            None => Err(RuntimeError::Unexpected(
                "the root folder does not exist.".to_string(),
            )),
        }
    } else {
        Ok(file_pb.clone())
    }
}
