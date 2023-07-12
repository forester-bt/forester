#[macro_use]
extern crate log;

use crate::runtime::rtree::RuntimeTree;
use crate::runtime::{RtOk, RtResult};
use crate::tree::project::Project;
use log::LevelFilter;
use std::fs;
use std::path::PathBuf;

pub mod runtime;
pub mod simulator;
pub mod tracer;
pub mod tree;
pub mod visualizer;

use crate::runtime::RuntimeError;
#[cfg(test)]
mod tests;

pub fn read_file(pb: &PathBuf) -> RtResult<String> {
    Ok(fs::read_to_string(pb)
        .map_err(|e| RuntimeError::IOError(format!("error:{}, file:{:?}", e.to_string(), pb)))?)
}

pub(crate) fn get_pb(file: &String, root: PathBuf) -> PathBuf {
    let file_pb = PathBuf::from(file);
    if file_pb.is_relative() {
        let mut full_path = root;
        full_path.push(file_pb);
        full_path
    } else {
        file_pb
    }
}
