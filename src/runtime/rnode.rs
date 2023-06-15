use crate::runtime::action::ActionName;
use crate::runtime::args::RtArgs;
use crate::runtime::TickResult;
use crate::tree::parser::ast::{Tree, TreeType};
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
pub enum RTreeType {
    Root,
    Parallel,
    Sequence,
    MSequence,
    RSequence,
    Fallback,
    RFallback,
}

pub type TreeName = String;
pub type RNodeId = usize;

#[derive(Clone)]
pub enum RNodeType {
    Action,
    Tree(RTreeType),
    Decorator(DecoratorType),
}

impl From<TreeType> for RNodeType {
    fn from(value: TreeType) -> Self {
        match value {
            TreeType::Root => RNodeType::Tree(RTreeType::Root),
            TreeType::Parallel => RNodeType::Tree(RTreeType::Parallel),
            TreeType::Sequence => RNodeType::Tree(RTreeType::Sequence),
            TreeType::MSequence => RNodeType::Tree(RTreeType::MSequence),
            TreeType::RSequence => RNodeType::Tree(RTreeType::RSequence),
            TreeType::Fallback => RNodeType::Tree(RTreeType::Fallback),
            TreeType::RFallback => RNodeType::Tree(RTreeType::RFallback),
            TreeType::Inverter => RNodeType::Decorator(DecoratorType::Inverter),
            TreeType::ForceSuccess => RNodeType::Decorator(DecoratorType::ForceSuccess),
            TreeType::ForceFail => RNodeType::Decorator(DecoratorType::ForceFail),
            TreeType::Repeat => RNodeType::Decorator(DecoratorType::Repeat),
            TreeType::Retry => RNodeType::Decorator(DecoratorType::Retry),
            TreeType::Timeout => RNodeType::Decorator(DecoratorType::Timeout),
            TreeType::Delay => RNodeType::Decorator(DecoratorType::Delay),
            TreeType::Impl => RNodeType::Action,
            TreeType::Cond => RNodeType::Action,
        }
    }
}

pub enum RNode {
    Leaf(ActionName, RtArgs),
    Tree(RTreeType, TreeName, RtArgs, Vec<RNodeId>),
    Decorator(DecoratorType, TreeName, RtArgs, RNodeId),
}

impl RNode {
    pub fn d_lambda(t: DecoratorType, child: RNodeId) -> Self {
        RNode::Decorator(t, "_".to_string(), RtArgs::default(), child)
    }
    pub fn lambda(t: RTreeType, children: Vec<RNodeId>) -> Self {
        RNode::Tree(t, "_".to_string(), RtArgs::default(), children)
    }
    pub fn root(name: TreeName, children: Vec<RNodeId>) -> Self {
        RNode::Tree(RTreeType::Root, name, RtArgs::default(), children)
    }
}

pub struct RNodeState {
    last_tick: TickResult,
}
