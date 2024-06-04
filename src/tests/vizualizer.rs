use crate::runtime::rtree::RuntimeTree;
use crate::tests::test_folder;
use crate::tree::project::Project;
use crate::visualizer::Visualizer;




#[test]
#[ignore]
fn manual() {
    let project_root = test_folder("plain_project").clone();
    let mut graph = project_root.clone();
    let project = Project::build("main.tree".to_string(), project_root).unwrap();

    let tree = RuntimeTree::build(project).unwrap().tree;

    graph.push("main.svg");
    let r = Visualizer::rt_tree_svg_to_file(&tree, graph).unwrap();
    assert!(r.is_empty());
}
