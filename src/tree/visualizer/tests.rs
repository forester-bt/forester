use std::path::PathBuf;
use crate::tree::project::Project;
use crate::tree::visualizer::Visualizer;

#[test]
fn smoke() {
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let mut project_root = root.clone();
    let mut graph = root.clone();
    project_root.push("tree/tests/plain_project");
    let project = Project::build("main.tree".to_string(), project_root).unwrap();

    let mut v = Visualizer::new(&project);
    graph.push("tree/graphs/graph.svg");
    v.to_svg_file(graph.to_str().unwrap().to_string()).unwrap();
}