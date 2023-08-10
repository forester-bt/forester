use crate::runtime::rtree::rnode::RNodeId;
use crate::runtime::rtree::RNode;
use crate::runtime::rtree::RuntimeTree;
use std::collections::HashMap;

pub struct RtTreeAnalyzer<'a> {
    tree: &'a RuntimeTree,
    parents: HashMap<RNodeId, RNodeId>,
}

impl<'a> RtTreeAnalyzer<'a> {
    pub fn new(tree: &'a RuntimeTree) -> Self {
        let mut parents = HashMap::new();
        for (id, node) in tree.iter() {
            for child in node.children() {
                parents.insert(child, id);
            }
        }

        Self { tree, parents }
    }

    pub fn parent(&self, id: &RNodeId) -> Option<&RNodeId> {
        self.parents.get(id)
    }
    pub fn find_by<F>(&self, filter: F) -> Option<RNodeId>
    where
        F: Fn(&RNode) -> bool,
    {
        self.tree
            .iter()
            .find_map(|(id, node)| if filter(node) { Some(id) } else { None })
    }
}
