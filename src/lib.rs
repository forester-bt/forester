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
use crate::runtime::rtree::{RuntimeTree, RuntimeTreeStarter};

pub mod runtime;
pub mod simulator;
pub mod tracer;
pub mod tree;
pub mod visualizer;
pub mod converter;

use crate::runtime::RuntimeError;
use crate::tree::project::Project;
use crate::tree::TreeError;

#[cfg(test)]
mod tests;

pub fn read_file(pb: &PathBuf) -> RtResult<String> {
    fs::read_to_string(pb)
        .map_err(|e| RuntimeError::IOError(format!("error:{e}, file:{}", pb.as_path().display())))
}

pub fn runtime_tree_default(
    root: PathBuf,
    file: Option<&String>,
    tree: Option<&String>,
    output: Option<&String>,
    output_ext:String
) -> Result<(RuntimeTreeStarter,PathBuf), TreeError> {
    let project = match (file, tree) {
        (Some(file), Some(tree)) => {
            Project::build_with_root(file.to_string(), tree.to_string(), root)
        }
        (Some(file), None) => Project::build(file.to_string(), root),
        _ => Project::build("main.tree".to_string(), root),
    }?;
    let output_pb = match output {
        Some(path) => get_pb(&PathBuf::from(path), &Some(project.root.clone()))?,
        None => {
            let mut output_name = PathBuf::from(project.main.0.clone());
            let _ = output_name.set_extension(output_ext);
            let mut new_output = project.root.clone();
            new_output.push(output_name);
            new_output
        }
    };
    let rt = RuntimeTree::build(project)?;
    Ok((rt, output_pb))
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
