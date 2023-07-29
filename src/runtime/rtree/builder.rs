use crate::args;
use crate::runtime::args::RtArgs;
use crate::runtime::rtree::rnode::{DecoratorType, FlowType, RNode, RNodeId, RNodeName};
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::RtResult;
use std::collections::{HashMap, HashSet};

pub struct RtTreeBuilder {
    root: RNodeId,
    max: RNodeId,
    nodes: HashMap<RNodeId, RNode>,
    actions: HashSet<String>,
}

impl RtTreeBuilder {
    pub fn build(self) -> (RuntimeTree, HashSet<String>) {
        (
            RuntimeTree {
                root: self.root,
                nodes: self.nodes,
            },
            self.actions,
        )
    }

    fn next(&mut self) -> RNodeId {
        self.max += 1;
        self.max
    }

    pub fn new() -> Self {
        Self {
            root: 0,
            max: 0,
            nodes: HashMap::new(),
            actions: HashSet::new(),
        }
    }

    fn process_child(&mut self, child: RtChild) -> RNodeId {
        match child {
            RtChild::Id(id) => id,
            RtChild::Node(nb) => self.add(nb),
        }
    }

    pub fn add(&mut self, node_b: RtNodeBuilder) -> RNodeId {
        match node_b {
            RtNodeBuilder::Leaf(n, args) => {
                let id = self.next();
                self.actions
                    .insert(n.name().unwrap_or(&"".to_string()).clone());
                self.nodes.insert(id, RNode::Leaf(n, args));
                id
            }
            RtNodeBuilder::Decorator(t, args, child) => {
                let id = self.process_child(*child);
                let d_id = self.next();
                self.nodes.insert(d_id, RNode::Decorator(t, args, id));
                d_id
            }
            RtNodeBuilder::Flow(t, name, args, children) => {
                let mut children_ids = vec![];
                for c in children {
                    children_ids.push(self.process_child(c));
                }
                let id = self.next();
                if t.is_root() {
                    self.root = id
                }
                self.nodes
                    .insert(id, RNode::Flow(t, name, args, children_ids));
                id
            }
        }
    }
}

pub enum RtNodeBuilder {
    Leaf(RNodeName, RtArgs),
    Decorator(DecoratorType, RtArgs, Box<RtChild>),
    Flow(FlowType, RNodeName, RtArgs, Vec<RtChild>),
}

impl RtNodeBuilder {
    pub fn leaf(name: RNodeName, args: RtArgs) -> Self {
        RtNodeBuilder::Leaf(name, args)
    }
    pub fn decorator(t: DecoratorType, args: RtArgs, child: RtChild) -> Self {
        RtNodeBuilder::Decorator(t, args, Box::new(child))
    }
    pub fn flow(t: FlowType, name: RNodeName, args: RtArgs, children: Vec<RtChild>) -> Self {
        RtNodeBuilder::Flow(t, name, args, children)
    }
}

pub enum RtChild {
    Id(RNodeId),
    Node(RtNodeBuilder),
}

impl From<RNodeId> for RtChild {
    fn from(value: RNodeId) -> Self {
        RtChild::Id(value)
    }
}

impl From<RtNodeBuilder> for RtChild {
    fn from(value: RtNodeBuilder) -> Self {
        RtChild::Node(value)
    }
}

#[cfg(test)]
mod tests {
    use crate::runtime::args::*;
    use crate::runtime::rtree::builder::*;
    use crate::runtime::rtree::rnode::RNodeName;
    use crate::tree::project::Project;
    use crate::visualizer::Visualizer;
    use crate::{action, arg, args, decorator, flow, node_name, rt_num, rt_str};
    use std::path::PathBuf;

    #[test]
    fn smoke() {
        let mut b = RtTreeBuilder::new();
        b.add(flow!(
            root node_name!(), args!();
            flow!(fallback node_name!(), args!(); action!()),
            flow!(sequence node_name!(), args!(arg!("a", rt_num!(i 1)));
                action!(node_name!("a")), action!()
            )

        ));
        let (tree, _) = b.build();
        Visualizer::svg_file(
            &tree,
            PathBuf::from(r#"C:\projects\forester\tools\cli\1.svg"#),
        )
        .unwrap();
    }
}
