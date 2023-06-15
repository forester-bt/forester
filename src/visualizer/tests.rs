use std::path::PathBuf;
use graphviz_rust::dot_generator::*;
use graphviz_rust::dot_structures::*;
use crate::tree::project::Project;
use crate::visualizer::Visualizer;

#[test]
fn manual() {
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let mut project_root = root.clone();
    let mut graph = root.clone();
    project_root.push("tree/tests/plain_project");
    let project = Project::build("main.tree".to_string(), project_root).unwrap();

    let mut v = Visualizer::new(&project);
    graph.push("tree/tests/plain_project/main.svg");
    v.to_svg_file(graph.to_str().unwrap().to_string()).unwrap();
}
#[test]
fn manual2() {
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let mut project_root = root.clone();
    let mut graph = root.clone();
    project_root.push("tree/tests/drive_robot");
    let project = Project::build("amr_simple.tree".to_string(), project_root).unwrap();

    let mut v = Visualizer::new(&project);
    graph.push("tree/tests/drive_robot/amr_simple.svg");
    v.to_svg_file(graph.to_str().unwrap().to_string()).unwrap();
}