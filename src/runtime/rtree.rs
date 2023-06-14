use std::collections::HashMap;
use crate::runtime::action::{ActionKeeper, ActionName};
use crate::runtime::args::RtArgs;
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::rnode::{RNode, RNodeId, RNodeState};
use crate::runtime::RuntimeError;
use crate::tree::project::Project;

pub struct TreeContext {
    bb: BlackBoard,

}

pub struct RuntimeTree {
    pub root: RNodeId,
    pub nodes: HashMap<RNodeId, RNode>,
    pub state: HashMap<RNodeId, RNodeState>,
}

impl RuntimeTree {
    pub fn build(project:Project) -> Result<RuntimeTree,RuntimeError>{
        let mut id = 0;

    }
}

