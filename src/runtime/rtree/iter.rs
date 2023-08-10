use crate::runtime::rtree::rnode::{RNode, RNodeId};
use crate::runtime::rtree::RuntimeTree;
use std::collections::VecDeque;

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
