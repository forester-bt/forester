use crate::runtime::rtree::rnode::RNodeId;
use crate::runtime::rtree::RNode;
use crate::runtime::rtree::RuntimeTree;
use std::collections::HashMap;

/// Runtime tree analyzer
/// Provides methods to analyze the runtime tree
/// and find nodes by some criteria
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
    /// Returns the parent of the node with the given id
    pub fn parent(&self, id: &RNodeId) -> Option<&RNodeId> {
        self.parents.get(id)
    }

    /// Returns the parent of the node with the given id
    pub fn find_by<F>(&self, filter: F) -> Option<RNodeId>
    where
        F: Fn(&RNode) -> bool,
    {
        self.tree
            .iter()
            .find_map(|(id, node)| if filter(node) { Some(id) } else { None })
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
    fn find_and_parent() {
        let mut rtb = RtTreeBuilder::new();

        let flow = flow!(fallback node_name!("root"), args!();
            flow!(sequence node_name!("seq"), args!();
                 action!(node_name!("action1"))
            ),
           action!(node_name!("action2"))
        );

        rtb.add_as_root(flow);
        let tree = rtb.build().unwrap().0;

        let analyzer = tree.analyze();
        let a1 = analyzer.find_by(|n| n.is_name("action1")).unwrap();
        let a2 = analyzer.find_by(|n| n.is_name("action2")).unwrap();
        let root = analyzer.find_by(|n| n.is_name("root")).unwrap();
        let seq = analyzer.find_by(|n| n.is_name("seq")).unwrap();

        assert_eq!(analyzer.parent(&a1), Some(&seq));
        assert_eq!(analyzer.parent(&a2), Some(&root));
        assert_eq!(analyzer.parent(&root), None);
    }
}
