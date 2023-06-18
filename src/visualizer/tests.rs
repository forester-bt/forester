use crate::runtime::rtree::RuntimeTree;
use crate::tree::project::Project;
use crate::visualizer::Visualizer;
use graphviz_rust::dot_generator::*;
use graphviz_rust::dot_structures::*;
use std::path::PathBuf;

#[test]
fn manual() {
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let mut project_root = root.clone();
    let mut graph = root.clone();
    project_root.push("tree/tests/plain_project");
    let project = Project::build("main.tree".to_string(), project_root).unwrap();

    let tree = RuntimeTree::build(project).unwrap();

    graph.push("tree/tests/plain_project/main.svg");
    let svg = graph.to_str().unwrap().to_string();
    let _ = Visualizer::svg_file(tree, svg).unwrap();
}
#[test]
fn manual2() {
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let mut project_root = root.clone();
    let mut graph = root.clone();
    project_root.push("tree/tests/drive_robot");
    let project = Project::build("amr_simple.tree".to_string(), project_root).unwrap();

    let tree = RuntimeTree::build(project).unwrap();

    graph.push("tree/tests/drive_robot/amr_simple.svg");
    let svg = graph.to_str().unwrap().to_string();
    let _ = Visualizer::svg_file(tree, svg).unwrap();
}
#[test]
fn manual3() {
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let mut project_root = root.clone();
    let mut graph = root.clone();
    project_root.push("tree/tests/ho_tree");
    let project = Project::build("main.tree".to_string(), project_root).unwrap();

    let tree = RuntimeTree::build(project).unwrap();

    graph.push("tree/tests/ho_tree/main.svg");
    let svg = graph.to_str().unwrap().to_string();
    let _ = Visualizer::svg_file(tree, svg).unwrap();
}
