mod actions;
mod decorator;
mod flow;

use crate::runtime::builder::ForesterBuilder;
use std::path::PathBuf;

pub fn fb(folder: &str) -> ForesterBuilder {
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    root.push("tree/tests");
    root.push(folder);

    let mut fb = ForesterBuilder::new();
    fb.main_file("main.tree".to_string());
    fb.root(root);

    fb
}
