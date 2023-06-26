use crate::runtime::action::keeper::ActionKeeper;
use crate::runtime::action::{Action, ActionName};
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::forester::Forester;
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::{RtResult, RuntimeError};
use crate::tree::project::{FileName, Project, TreeName};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

pub struct ForesterBuilder {
    actions: HashMap<ActionName, Action>,
    main_file: Option<FileName>,
    main: Option<TreeName>,
    root: Option<PathBuf>,
    bb: BlackBoard,
}

impl ForesterBuilder {
    pub fn new() -> Self {
        Self {
            actions: HashMap::default(),
            main_file: None,
            main: None,
            root: None,
            bb: BlackBoard::default(),
        }
    }

    pub fn register_action(&mut self, name: ActionName, action: Action) {
        self.actions.insert(name, action);
    }

    pub fn root(&mut self, root: PathBuf) {
        self.root = Some(root);
    }
    pub fn main_file(&mut self, main_file: FileName) {
        self.main_file = Some(main_file);
    }
    pub fn main_tree(&mut self, main_tree: TreeName) {
        self.main = Some(main_tree);
    }

    pub fn build(self) -> RtResult<Forester> {
        let project = match (self.main, self.root, self.main_file) {
            (None, Some(root), Some(mf)) => Project::build(mf, root)?,
            (Some(mt), Some(root), Some(mf)) => Project::build_with_root(mf, mt, root)?,
            _ => {
                return Err(RuntimeError::UnImplementedAction(format!(
                    "not enough arguments to initialize the project"
                )))
            }
        };

        Ok(Forester::new(
            RuntimeTree::build(project)?,
            BlackBoard::default(),
            ActionKeeper::new(self.actions),
        ))
    }
}
