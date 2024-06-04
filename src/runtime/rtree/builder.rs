use crate::runtime::args::RtArgs;
use crate::runtime::rtree::rnode::{DecoratorType, FlowType, RNode, RNodeId, RNodeName};
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::{RtResult, RuntimeError};
use std::collections::{HashMap, HashSet};

/// The builder for the runtime tree.
/// It is used to build the runtime tree from the tree definition.
/// The builder is used in the tree definition macros.
///
/// # Example
/// ```
///     use forester_rs::runtime::args::RtArgs;
///     use forester_rs::runtime::rtree::builder::RtNodeBuilder;
///     use forester_rs::runtime::rtree::builder::RtTreeBuilder;
///     use forester_rs::runtime::rtree::rnode::FlowType;
///     use forester_rs::runtime::rtree::rnode::RNodeName;
///     use forester_rs::*;
///
///     #[test]
///     fn tree() {
///         let mut rtb = RtTreeBuilder::new();
///
///         let flow = flow!(fallback node_name!("root"), args!();
///             flow!(sequence node_name!("seq"), args!();
///                  action!(node_name!("action1"))
///             ),
///            action!(node_name!("action2"))
///         );
///
///         rtb.add_as_root(flow);
///         let tree = rtb.build().unwrap().0;
///         
///     }
/// ```
#[derive(Debug)]
pub struct RtTreeBuilder {
    pub root: Option<RNodeId>,
    pub max: RNodeId,
    pub nodes: HashMap<RNodeId, RNode>,
    pub actions: HashSet<String>,
}

impl RtTreeBuilder {
    /// Builds the runtime tree from the builder
    /// Returns the runtime tree and the set of action names
    pub fn build(self) -> RtResult<(RuntimeTree, HashSet<String>)> {
        let root = self
            .root
            .ok_or(RuntimeError::uex("root should be presented".to_string()))?;
        Ok((
            RuntimeTree {
                root,
                nodes: self.nodes,
            },
            self.actions,
        ))
    }

    fn next(&mut self) -> RNodeId {
        self.max += 1;
        self.max
    }

    pub fn new() -> Self {
        Self {
            root: None,
            max: 0,
            nodes: HashMap::new(),
            actions: HashSet::new(),
        }
    }
    /// Creates the builder with the initial root id value
    pub fn new_from(from: RNodeId) -> Self {
        Self {
            root: None,
            max: from,
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
    /// Generate a new id and adds the node to the builder  and returns its id
    pub fn add(&mut self, node_b: RtNodeBuilder) -> RNodeId {
        let id = self.next();
        self.set(node_b, id);
        id
    }

    /// Generate a new id and adds the node to the builder , sets it as root and returns its id
    pub fn add_as_root(&mut self, node_b: RtNodeBuilder) -> RNodeId {
        let id = self.add(node_b);
        self.root = Some(id.clone());
        id
    }

    /// sets the root node id
    pub fn root(&mut self, id: RNodeId) {
        self.root = Some(id);
    }

    /// Sets the node with the given id
    pub fn set(&mut self, node_b: RtNodeBuilder, id: RNodeId) {
        match node_b {
            RtNodeBuilder::Leaf(n, args) => {
                self.actions
                    .insert(n.name().unwrap_or(&"".to_string()).clone());
                self.nodes.insert(id, RNode::Leaf(n, args));
            }
            RtNodeBuilder::Decorator(t, args, child) => {
                let cid = self.process_child(*child);
                self.nodes.insert(id, RNode::Decorator(t, args, cid));
            }
            RtNodeBuilder::Flow(t, name, args, children) => {
                let mut children_ids = vec![];
                for c in children {
                    children_ids.push(self.process_child(c));
                }
                if t.is_root() {
                    self.root = Some(id)
                }
                self.nodes
                    .insert(id, RNode::Flow(t, name, args, children_ids));
            }
        }
    }
    /// Sets the node with the given id as root
    pub fn set_as_root(&mut self, node_b: RtNodeBuilder, id: RNodeId) {
        self.set(node_b, id.clone());
        self.root = Some(id)
    }
}

/// The builder for the runtime tree node.
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
/// The child of the runtime tree node.
/// It can be either the id of the node or the node builder that will be transformed later
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
    
    
    use crate::{action, arg, args, decorator, flow, node_name, rt_num};
    

    #[test]
    fn smoke() {
        let mut b = RtTreeBuilder::new();
        b.add(flow!(
            root node_name!(), args!();
            flow!(fallback node_name!(), args!(); action!()),
            flow!(sequence node_name!(), args!(arg!("a", rt_num!(i 1)));
                action!(node_name!("a")),
                decorator!(inverter args!(), action!())
            )

        ));
        let (tree, _) = b.build().unwrap();

        let nodes = tree
            .iter()
            .map(|(id, node)| {
                let node = match node {
                    RNode::Leaf(_n, _args) => "action".to_string(),
                    RNode::Flow(t, _n, _args, _children) => {
                        format!("{t}")
                    }
                    RNode::Decorator(t, _args, _child) => {
                        format!("{t}")
                    }
                };
                (id, node)
            })
            .collect::<Vec<_>>();
        assert_eq!(
            nodes,
            vec![
                (1, "root".to_string()),
                (2, "fallback".to_string()),
                (4, "sequence".to_string()),
                (3, "action".to_string()),
                (5, "action".to_string()),
                (6, "inverter".to_string()),
                (7, "action".to_string())
            ]
        )
    }
}
