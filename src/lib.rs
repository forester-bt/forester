#[macro_use]
extern crate log;

use log::LevelFilter;

pub mod runtime;
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

#[cfg(test)]
pub mod test_utils {
    use crate::runtime::rtree::RuntimeTree;
    use crate::tree::project::Project;
    use std::path::PathBuf;

    pub fn test_tree(root_dir: &str, root_file: &str) -> RuntimeTree {
        let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        root.push("tree/tests");
        root.push(root_dir);
        let project = Project::build(root_file.to_string(), root).unwrap();
        RuntimeTree::build(project).unwrap()
    }
}
