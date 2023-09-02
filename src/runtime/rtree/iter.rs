use crate::runtime::rtree::rnode::{RNode, RNodeId};
use crate::runtime::rtree::RuntimeTree;
use std::collections::VecDeque;
/// simple bfs iterator over the tree
pub struct RtTreeBfsIter<'a> {
    pub queue: VecDeque<RNodeId>,
    pub tree: &'a RuntimeTree,
}

impl<'a> Iterator for RtTreeBfsIter<'a> {
    type Item = (RNodeId, &'a RNode);

    fn next(&mut self) -> Option<Self::Item> {
        match self.queue.pop_front() {
            None => None,
            Some(id) => match self.tree.nodes.get(&id) {
                None => None,
                Some(node) => {
                    match &node {
                        RNode::Leaf(_, _) => {}
                        RNode::Flow(_, _, _, children) => self.queue.extend(children.iter()),
                        RNode::Decorator(_, _, child) => self.queue.push_back(*child),
                    }
                    Some((id, node))
                }
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::runtime::args::RtArgs;
    use crate::runtime::rtree::builder::RtNodeBuilder;
    use crate::runtime::rtree::builder::RtTreeBuilder;
    use crate::runtime::rtree::rnode::FlowType;
    use crate::runtime::rtree::rnode::RNodeName;
    use crate::*;
    #[test]
    fn smoke() {
        let flow = flow!(fallback node_name!("main"), args!();
            flow!(sequence node_name!("root"), args!();
                 action!(node_name!("action1"))
            ),
           action!(node_name!("action2"))
        );
        let mut rtb = RtTreeBuilder::new();
        rtb.add_as_root(flow);
        let tree = rtb.build().unwrap().0;
        let elems: Vec<_> = tree
            .iter()
            .map(|(id, node)| (id, node.name().unwrap().name().unwrap().clone()))
            .collect();

        assert_eq!(
            elems,
            vec![
                (1, "main".to_string()),
                (2, "root".to_string()),
                (4, "action2".to_string()),
                (3, "action1".to_string()),
            ]
        );
    }
}
