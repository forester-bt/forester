use crate::runtime::action::ActionName;
use crate::runtime::args::RtArgs;
use crate::runtime::{RuntimeErrorCause, TickResult};
use crate::tree::parser::ast::{Tree, TreeType};

pub type RNodeId = usize;
pub type Name = String;
pub type Alias = String;

#[derive(Clone)]
pub enum DecoratorType {
    Inverter,
    ForceSuccess,
    ForceFail,
    Repeat,
    Retry,
    Timeout,
    Delay,
}
#[derive(Clone)]
pub enum FlowType {
    Root,
    Parallel,
    Sequence,
    MSequence,
    RSequence,
    Fallback,
    RFallback,
}

impl TryFrom<TreeType> for DecoratorType {
    type Error = RuntimeErrorCause;

    fn try_from(value: TreeType) -> Result<Self, Self::Error> {
        match value {
            TreeType::Inverter => Ok(DecoratorType::Inverter),
            TreeType::ForceSuccess => Ok(DecoratorType::ForceSuccess),
            TreeType::ForceFail => Ok(DecoratorType::ForceFail),
            TreeType::Repeat => Ok(DecoratorType::Repeat),
            TreeType::Retry => Ok(DecoratorType::Retry),
            TreeType::Timeout => Ok(DecoratorType::Timeout),
            TreeType::Delay => Ok(DecoratorType::Delay),
            e => Err(RuntimeErrorCause::un(format!(
                "unexpected type {} for decorator",
                e.to_string()
            ))),
        }
    }
}

impl TryFrom<TreeType> for FlowType {
    type Error = RuntimeErrorCause;

    fn try_from(value: TreeType) -> Result<Self, Self::Error> {
        match value {
            TreeType::Root => Ok(FlowType::Root),
            TreeType::Parallel => Ok(FlowType::Parallel),
            TreeType::Sequence => Ok(FlowType::Sequence),
            TreeType::MSequence => Ok(FlowType::MSequence),
            TreeType::RSequence => Ok(FlowType::RSequence),
            TreeType::Fallback => Ok(FlowType::Fallback),
            TreeType::RFallback => Ok(FlowType::RFallback),
            e => Err(RuntimeErrorCause::un(format!(
                "unexpected type {} for flow",
                e.to_string()
            ))),
        }
    }
}

pub enum RNodeName {
    Lambda,
    Name(Name),
    Alias(Name, Alias),
}

pub enum RNode {
    Leaf(RNodeName, RtArgs),
    Flow(FlowType, RNodeName, RtArgs, Vec<RNodeId>),
    Decorator(DecoratorType, RtArgs, RNodeId),
}

impl RNode {
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
    pub fn flow_alias(
        f: FlowType,
        name: Name,
        alias: Alias,
        args: RtArgs,
        children: Vec<RNodeId>,
    ) -> Self {
        RNode::Flow(f, RNodeName::Name(name), args, children)
    }
}

pub struct RNodeState {
    last_tick: TickResult,
}
