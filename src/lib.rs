#[macro_use]
extern crate log;

use crate::runtime::rtree::RuntimeTree;
use crate::runtime::RtOk;
use crate::tree::project::Project;
use log::LevelFilter;
use std::path::PathBuf;

pub mod runtime;
pub mod tracer;
pub mod tree;
pub mod visualizer;

fn turn_on_logs() {
    let _ = env_logger::builder()
        .is_test(true)
        .filter_level(LevelFilter::max())
        .format_timestamp(None)
        .format_level(false)
        .try_init();
}
