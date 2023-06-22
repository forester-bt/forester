use crate::runtime::action::keeper::ActionKeeper;
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::RuntimeErrorCause;
use crate::tree::project::Project;

pub struct Forester {
    pub tree: RuntimeTree,
    pub bb: BlackBoard,
    pub keeper: ActionKeeper,
}
