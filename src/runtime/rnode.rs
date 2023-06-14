use crate::runtime::action::ActionName;
use crate::runtime::args::RtArgs;
use crate::runtime::TickResult;

pub enum DecoratorType {
    Inverter,
    ForceSuccess,
    ForceFail,
    Repeat,
    Retry,
    Timeout,
    Delay,
}
pub enum RNodeType {
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

pub enum RNode {
    Leaf(ActionName, RtArgs),
    Tree(RNodeType, TreeName, RtArgs, Vec<RNodeId>),
    Decorator(DecoratorType, TreeName, RtArgs, RNodeId),
}


pub struct RNodeState {
    last_tick:TickResult
}