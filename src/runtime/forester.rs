use crate::runtime::action::ActionKeeper;
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::RuntimeErrorCause;
use crate::tree::project::Project;

pub struct Forester {
    pub r_tree: RuntimeTree,
    pub bb: BlackBoard,
    pub acts: ActionKeeper,
}

// impl Forester {
//     pub fn init(project: Project) -> Result<Self, RuntimeError> {
//
//
//
//
//     }
// }
