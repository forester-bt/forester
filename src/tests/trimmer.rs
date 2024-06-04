use crate::action;
use crate::runtime::action::{Impl};
use crate::runtime::args::RtArgs;
use crate::runtime::builder::ForesterBuilder;

use crate::runtime::rtree::builder::{RtNodeBuilder, RtTreeBuilder};
use crate::runtime::rtree::rnode::RNodeName;

use crate::runtime::trimmer::task::{RtTreeTrimTask, TrimTask};
use crate::runtime::trimmer::{RequestBody, TreeSnapshot, TrimRequest};
use crate::runtime::{RtResult, TickResult};
use crate::simulator::actions::SimAction;
use crate::tests::{test_folder, turn_on_logs};
use crate::tracer::{Tracer, TracerConfig};

use crate::*;


#[test]
fn smoke() {
    turn_on_logs();

    let root = test_folder("trimmer/smoke");

    let mut fb = ForesterBuilder::from_fs();

    fb.tracer(Tracer::create(TracerConfig::in_file(root.clone().join("main.log"), None)).unwrap());
    fb.main_file("main.tree".to_string());
    fb.root(root.clone());

    let mut forester = fb.build().unwrap();
    forester.add_trim_task(TrimTask::rt_tree(Test));
    let result = forester.run_until(Some(100)).unwrap();
    assert_eq!(result, TickResult::Success);
}

struct Test;

impl RtTreeTrimTask for Test {
    fn process(&self, snapshot: TreeSnapshot<'_>) -> RtResult<TrimRequest> {
        if snapshot.tick < 90 {
            Ok(TrimRequest::Skip)
        } else {
            let tree = snapshot.tree;
            let id = tree
                .nodes
                .iter()
                .find(|(_, n)| {
                    n.name()
                        .and_then(|n| n.name().ok())
                        .filter(|n| n.as_str() == "fail_empty")
                        .is_some()
                })
                .map(|(id, _)| id)
                .unwrap();

            let mut rtb = RtTreeBuilder::new_from(tree.max_id() + 1);
            rtb.set_as_root(action!(node_name!("success")), id.clone());

            Ok(TrimRequest::attempt(RequestBody::new(
                rtb,
                Default::default(),
            )))
        }
    }
}

#[test]
fn naive() {
    turn_on_logs();
    let root = test_folder("trimmer/naive");

    let mut fb = ForesterBuilder::from_fs();

    fb.tracer(Tracer::create(TracerConfig::in_file(root.clone().join("main.log"), None)).unwrap());
    fb.main_file("main.tree".to_string());
    fb.root(root.clone());

    fb.register_sync_action("pick", SimAction::Random(100));
    fb.register_sync_action("validate", SimAction::Success(100));
    fb.register_sync_action("place", SimAction::Success(100));

    let mut forester = fb.build().unwrap();
    for (n, i) in forester.tree.iter() {
        println!("{n} {:?}", i)
    }
    forester.add_trim_task(TrimTask::rt_tree(Test));
    let result = forester.run_until(Some(100)).unwrap();
    assert_eq!(result, TickResult::Success);
}

struct SmokeTest;

impl RtTreeTrimTask for SmokeTest {
    fn process(&self, snapshot: TreeSnapshot<'_>) -> RtResult<TrimRequest> {
        if snapshot.tick < 20 {
            Ok(TrimRequest::Skip)
        } else {
            let tree = snapshot.tree;
            for (id, node) in tree.iter() {
                println!("{id} {:?}", node)
            }
            Ok(TrimRequest::Reject)
        }
    }
}
