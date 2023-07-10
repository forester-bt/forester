mod actions;
mod decorator;
mod flow;
mod project;
mod rtree;
mod simulator;
mod tracer;
mod vizualizer;

use crate::runtime::builder::ForesterBuilder;
use log::LevelFilter;
use std::path::PathBuf;

pub fn turn_on_logs() {
    let _ = env_logger::builder()
        .is_test(true)
        .filter_level(LevelFilter::max())
        .format_timestamp(None)
        .format_level(false)
        .try_init();
}

pub fn test_folder(path_in: &str) -> PathBuf {
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    root.push("tree/tests");
    root.push(path_in);
    root
}

pub fn fb(folder: &str) -> ForesterBuilder {
    let mut root = test_folder(folder);

    let mut fb = ForesterBuilder::new();
    fb.main_file("main.tree".to_string());
    fb.root(root);

    fb
}
