use crate::exporter::Exporter;
use crate::exporter::ros_nav::RosNavBTree;
use crate::runtime::rtree::RuntimeTree;
use crate::tests::{fb, test_folder};
use crate::tree::project::Project;

#[test]
fn smoke() {
    let mut fb = test_folder("ros/nav/smoke");

    let project = Project::build("main.tree".to_string(), fb.clone()).unwrap();
    let tree = RuntimeTree::build(project).unwrap().tree;

    fb.push("test.xml");
    let exporter = RosNavBTree::new(fb);
    let mut writer = exporter.writer().unwrap();
    let _ = exporter.export(&mut writer, &tree).unwrap();
}