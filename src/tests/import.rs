use std::collections::HashMap;
use crate::runtime::args::RtArgs;
use crate::runtime::rtree::rnode::FlowType::{Root, Sequence};
use crate::runtime::rtree::rnode::RNode::{Flow, Leaf};
use crate::runtime::rtree::rnode::RNodeName::{Alias, Lambda, Name};
use crate::tests::fb;

#[test]
fn smoke() {
    let tree = fb("import/path").build().unwrap().tree;
    assert_eq!(tree.nodes, HashMap::from_iter(
        vec![
            (4, Leaf(Alias("action".to_string(), "action1".to_string(), "nested/nested2.tree".to_string()), RtArgs(vec![]))),
            (2, Flow(Sequence, Lambda, RtArgs(vec![]), vec![3, 4, 5])),
            (1, Flow(Root, Name("main".to_string(), "main.tree".to_string()), RtArgs(vec![]), vec![2])),
            (3, Leaf(Name("action".to_string(), "nested/nested2.tree".to_string()), RtArgs(vec![]))),
            (5, Leaf(Name("h".to_string(), "../util.tree".to_string()), RtArgs(vec![]))),
        ]
    ))
}