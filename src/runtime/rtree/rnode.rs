use crate::runtime::args::RtArgs;
use crate::runtime::{RtResult, RuntimeError, TickResult};
use crate::tree::parser::ast::TreeType;

use crate::tree::{cerr, TreeError};
use strum_macros::Display;
use strum_macros::EnumString;

pub type RNodeId = usize;
pub type Name = String;
pub type Alias = String;

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
#[derive(Debug, PartialEq)]
pub enum RNodeName {
    Lambda,
    Name(Name),
    Alias(Name, Alias),
}

impl RNodeName {
    pub fn name(&self) -> RtResult<&Name> {
        match self {
            RNodeName::Lambda => Err(RuntimeError::uex(
                "unexpected: lambda is in the unexpected place, the named tree is supposed to be."
                    .to_string(),
            )),
            RNodeName::Name(n) => Ok(n),
            RNodeName::Alias(n, _) => Ok(n),
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum RNode {
    Leaf(RNodeName, RtArgs),
    Flow(FlowType, RNodeName, RtArgs, Vec<RNodeId>),
    Decorator(DecoratorType, RtArgs, RNodeId),
}

impl RNode {
    pub fn name(&self) -> Option<&RNodeName> {
        match self {
            RNode::Leaf(n, _) | RNode::Flow(_, n, _, _) => Some(n),
            RNode::Decorator(_, _, _) => None,
        }
    }

    pub fn decorator(t: DecoratorType, args: RtArgs, child: RNodeId) -> Self {
        RNode::Decorator(t, args, child)
    }

    pub fn lambda(t: FlowType, children: Vec<RNodeId>) -> Self {
        RNode::Flow(t, RNodeName::Lambda, RtArgs::default(), children)
    }
    pub fn root(name: Name, children: Vec<RNodeId>) -> Self {
        RNode::Flow(
            FlowType::Root,
            RNodeName::Name(name),
            RtArgs::default(),
            children,
        )
    }
    pub fn flow(f: FlowType, name: Name, args: RtArgs, children: Vec<RNodeId>) -> Self {
        RNode::Flow(f, RNodeName::Name(name), args, children)
    }
    pub fn action(name: Name, args: RtArgs) -> Self {
        RNode::Leaf(RNodeName::Name(name), args)
    }
    pub fn action_alias(name: Name, alias: Alias, args: RtArgs) -> Self {
        RNode::Leaf(RNodeName::Alias(name, alias), args)
    }
    pub fn flow_alias(
        f: FlowType,
        name: Name,
        alias: Alias,
        args: RtArgs,
        children: Vec<RNodeId>,
    ) -> Self {
        RNode::Flow(f, RNodeName::Alias(name, alias), args, children)
    }
}
