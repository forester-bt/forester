use crate::runtime::args::RtArgs;
use crate::runtime::{RtResult, RuntimeError};
use crate::tree::parser::ast::TreeType;

use crate::tree::{cerr, TreeError};
use strum_macros::Display;
use strum_macros::EnumString;

pub type RNodeId = usize;
pub type Name = String;
pub type Alias = String;
pub type Path = String;

#[derive(Display, Debug, Clone, Copy, Eq, PartialEq, EnumString)]
#[strum(serialize_all = "snake_case")]
pub enum DecoratorType {
    Inverter,
    ForceSuccess,
    ForceFail,
    Repeat,
    Retry,
    Timeout,
    Delay,
}

#[derive(Display, Debug, Clone, Copy, Eq, PartialEq, EnumString)]
#[strum(serialize_all = "snake_case")]
pub enum FlowType {
    Root,
    Parallel,
    Sequence,
    MSequence,
    RSequence,
    Fallback,
    RFallback,
}

impl FlowType {
    pub fn is_root(&self) -> bool {
        matches!(self, FlowType::Root)
    }
    pub fn is_par(&self) -> bool {
        matches!(self, FlowType::Parallel)
    }
}

impl TryFrom<TreeType> for DecoratorType {
    type Error = TreeError;

    fn try_from(value: TreeType) -> Result<Self, Self::Error> {
        match value {
            TreeType::Inverter => Ok(DecoratorType::Inverter),
            TreeType::ForceSuccess => Ok(DecoratorType::ForceSuccess),
            TreeType::ForceFail => Ok(DecoratorType::ForceFail),
            TreeType::Repeat => Ok(DecoratorType::Repeat),
            TreeType::Retry => Ok(DecoratorType::Retry),
            TreeType::Timeout => Ok(DecoratorType::Timeout),
            TreeType::Delay => Ok(DecoratorType::Delay),
            e => Err(cerr(format!("unexpected type {e} for decorator"))),
        }
    }
}

impl TryFrom<TreeType> for FlowType {
    type Error = TreeError;

    fn try_from(value: TreeType) -> Result<Self, Self::Error> {
        match value {
            TreeType::Root => Ok(FlowType::Root),
            TreeType::Parallel => Ok(FlowType::Parallel),
            TreeType::Sequence => Ok(FlowType::Sequence),
            TreeType::MSequence => Ok(FlowType::MSequence),
            TreeType::RSequence => Ok(FlowType::RSequence),
            TreeType::Fallback => Ok(FlowType::Fallback),
            TreeType::RFallback => Ok(FlowType::RFallback),
            e => Err(cerr(format!("unexpected type {e} for flow"))),
        }
    }
}

/// A node name can be a lambda, a name or an alias.
/// An alias is a name that is used to refer to a node in the import
#[derive(Debug, PartialEq, Clone)]
pub enum RNodeName {
    Lambda,
    Name(Name, Path),
    Alias(Name, Alias, Path),
}

impl RNodeName {
    pub fn name(&self) -> RtResult<&Name> {
        match self {
            RNodeName::Lambda => Err(RuntimeError::uex(
                "unexpected: lambda is in the unexpected place, the named tree is supposed to be."
                    .to_string(),
            )),
            RNodeName::Name(n,_) => Ok(n),
            RNodeName::Alias(n, _, _) => Ok(n),
        }
    }
    pub fn has_name(&self) -> bool {
        match self {
            RNodeName::Lambda => false,
            RNodeName::Name(_,_) => true,
            RNodeName::Alias(_, _,_) => true,
        }
    }
}

/// A runtime node is a node that is used to execute a tree.
/// It can be a leaf, a flow or a decorator.
/// A leaf is a node that executes an action.
/// A flow is a node that executes a flow of nodes.
/// A decorator is a node that executes a child and decorates the result.
#[derive(Debug, PartialEq)]
pub enum RNode {
    Leaf(RNodeName, RtArgs),
    Flow(FlowType, RNodeName, RtArgs, Vec<RNodeId>),
    Decorator(DecoratorType, RtArgs, RNodeId),
}

impl RNode {
    /// check if the node has the given name
    pub fn is_name(&self, name: &str) -> bool {
        self.name()
            .and_then(|n| n.name().ok())
            .filter(|action| action.as_str() == name)
            .is_some()
    }
    /// check if the node has the given type
    pub fn is_flow(&self, tpe: &FlowType) -> bool {
        match self {
            RNode::Flow(t, ..) if t == tpe => true,
            _ => false,
        }
    }
    /// check if the node is a leaf
    pub fn is_action(&self) -> bool {
        match self {
            RNode::Leaf(..) => true,
            _ => false,
        }
    }

    /// check if the node has the given type
    pub fn is_decorator(&self, tpe: &DecoratorType) -> bool {
        match self {
            RNode::Decorator(t, ..) if t == tpe => true,
            _ => false,
        }
    }

    /// returns the args of the node
    pub fn args(&self) -> RtArgs {
        match self {
            RNode::Leaf(_, args) | RNode::Flow(_, _, args, _) | RNode::Decorator(_, args, _) => {
                args.clone()
            }
        }
    }

    /// returns the name of the node
    pub fn name(&self) -> Option<&RNodeName> {
        match self {
            RNode::Leaf(n, _) | RNode::Flow(_, n, _, _) => Some(n),
            RNode::Decorator(_, _, _) => None,
        }
    }

    /// returns the children ids of the node
    pub fn children(&self) -> Vec<RNodeId> {
        match self {
            RNode::Leaf(_, _) => vec![],
            RNode::Flow(_, _, _, children) => children.clone(),
            RNode::Decorator(_, _, child) => vec![*child],
        }
    }

    pub fn decorator(t: DecoratorType, args: RtArgs, child: RNodeId) -> Self {
        RNode::Decorator(t, args, child)
    }

    pub fn lambda(t: FlowType, children: Vec<RNodeId>) -> Self {
        RNode::Flow(t, RNodeName::Lambda, RtArgs::default(), children)
    }
    pub fn root(name: Name,p:Path, children: Vec<RNodeId>) -> Self {
        RNode::Flow(
            FlowType::Root,
            RNodeName::Name(name,p),
            RtArgs::default(),
            children,
        )
    }
    pub fn flow(f: FlowType, name: Name, p:Path, args: RtArgs, children: Vec<RNodeId>) -> Self {
        RNode::Flow(f, RNodeName::Name(name,p), args, children)
    }
    pub fn action(name: Name,path:Path, args: RtArgs) -> Self {
        RNode::Leaf(RNodeName::Name(name,path), args)
    }
    pub fn action_alias(name: Name, path:Path, alias: Alias, args: RtArgs) -> Self {
        RNode::Leaf(RNodeName::Alias(name, alias, path), args)
    }
    pub fn flow_alias(
        f: FlowType,
        name: Name,
        path:Path,
        alias: Alias,
        args: RtArgs,
        children: Vec<RNodeId>,
    ) -> Self {
        RNode::Flow(f, RNodeName::Alias(name, alias, path), args, children)
    }
}
